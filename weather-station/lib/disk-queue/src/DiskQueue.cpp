/*
 * Copyright (c) 2021 Particle Industries, Inc.  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "DiskQueue.h"
#include <fcntl.h>
#include <dirent.h>

// TODO
// * peekFront() after peekFrontSize() problem
// * Handle pushBack() policy for DiskQueuePolicy::FifoDeleteNew

int DiskQueue::start(const char* path, DiskQueuePolicy policy) {
    // Check if already running
    CHECK_FALSE(_running, SYSTEM_ERROR_INVALID_STATE);

    // The lock here is to prevent the reader and writer from running
    const std::lock_guard<RecursiveMutex> lock(_lock);

    int ret = SYSTEM_ERROR_UNKNOWN;
    do {
        if (!dirExists(path) && mkdir(path, 0775)) {
            ret = SYSTEM_ERROR_FILE;
            break;
        }

        _path = String(path) + "/";

        // Create a list of all filenames that may contain previously saved data
        getFilenames(path);

        _policy = policy;
        _running = true;

        return SYSTEM_ERROR_NONE;
    } while (false);

    // Cleanup if errors
    cleanup();
    return ret;
}

int DiskQueue::stop() {
    // The lock here is to prevent the reader and writer from running
    const std::lock_guard<RecursiveMutex> lock(_lock);

    _running = false;

    // Close all files and
    cleanupFiles();

    return SYSTEM_ERROR_NONE;
}

void DiskQueue::setDiskLimit(size_t size) {
    // The lock here is to prevent disk limit updates from affecting the reader and writer
    const std::lock_guard<RecursiveMutex> lock(_lock);

    _diskLimit = size;
}

int DiskQueue::getReadPolicyIndex(DiskQueuePolicy policy) {
    return 0; // Will always be the first for now
}

int DiskQueue::getWriteOverflowPolicyIndex(DiskQueuePolicy policy) {
    int ret = 0;

    switch (policy) {
    case DiskQueuePolicy::FifoDeleteOld:
        ret = 0;
        break;

    case DiskQueuePolicy::FifoDeleteNew:
        ret = _fileList.size() - 1;  // Will not be < zero
        break;
    }

    return ret;
}

size_t DiskQueue::peekFrontSize() {
    CHECK_TRUE(_running, 0);

    // The lock here is to prevent the writer from catching up with the reader
    const std::lock_guard<RecursiveMutex> lock(_lock);

    size_t size = 0;

    while (true) {
        if (_fileList.isEmpty()) {
            break; // Nothing available
        }

        auto entry = _fileList.first(); // TODO: Apply policy
        unsigned long fileN = entry->n;
        String filename = _path + String(fileN);

        auto fd = open(filename.c_str(), O_RDWR, 0664);
        if (0 > fd) {
            // File open is unsuccessful so remove file and continue
            unlink(filename.c_str());
            removeFileNode(getReadPolicyIndex(_policy));
            continue;
        }

        // Get the file header
        QueueFileHeader fileHeader = {};
        auto ret = read(fd, &fileHeader, sizeof(fileHeader));
        if (((int)sizeof(fileHeader) > ret) ||
            (QueueFileMagic != fileHeader.magic) ||
            (QueueFileVersion1 != fileHeader.version)) {

            close(fd);
            unlink(filename.c_str());
            removeFileNode(getReadPolicyIndex(_policy));
            continue;
        }

        // Get the item header
        QueueItemHeader itemHeader = {};
        ret = read(fd, &itemHeader, sizeof(itemHeader));
        if (((int)sizeof(itemHeader) > ret) ||
            (QueueItemMagic != itemHeader.magic) ||
            (0 == (ItemFlagActive & itemHeader.flags))) {

            close(fd);
            unlink(filename.c_str());
            removeFileNode(getReadPolicyIndex(_policy));
            continue;
        }

        // Everything was successful
        close(fd);
        size = (size_t)itemHeader.length;
        break;
    }

    return size;
}

// TODO: given size may be derived from peekFrontSize but this function may advance forward to another
//       entry until successful
bool DiskQueue::peekFront(uint8_t* data, size_t& size) {
    CHECK_TRUE(_running, false);

    // The lock here is to prevent the writer from catching up with the reader
    const std::lock_guard<RecursiveMutex> lock(_lock);

    auto success = false;

    while (true) {
        if (_fileList.isEmpty()) {
            size = 0;
            break; // Nothing available
        }

        auto entry = _fileList.first(); // TODO: Apply policy
        unsigned long fileN = entry->n;
        String filename = _path + String(fileN);

        auto fd = open(filename.c_str(), O_RDWR, 0664);
        if (0 > fd) {
            // File open is unsuccessful so remove file and continue
            unlink(filename.c_str());
            removeFileNode(getReadPolicyIndex(_policy));
            continue;
        }

        // Get the file header
        QueueFileHeader fileHeader = {};
        auto ret = read(fd, &fileHeader, sizeof(fileHeader));
        if (((int)sizeof(fileHeader) > ret) ||
            (QueueFileMagic != fileHeader.magic) ||
            (QueueFileVersion1 != fileHeader.version)) {

            close(fd);
            unlink(filename.c_str());
            removeFileNode(getReadPolicyIndex(_policy));
            continue;
        }

        // Get the item header
        QueueItemHeader itemHeader = {};
        ret = read(fd, &itemHeader, sizeof(itemHeader));
        if (((int)sizeof(itemHeader) > ret) ||
            (QueueItemMagic != itemHeader.magic) ||
            (0 == (ItemFlagActive & itemHeader.flags))) {

            close(fd);
            unlink(filename.c_str());
            removeFileNode(getReadPolicyIndex(_policy));
            continue;
        }

        // Get the data
        auto toRead = std::min<size_t>(size, (size_t)itemHeader.length);
        ret = read(fd, data, toRead);
        if ((int)toRead > ret) {
            close(fd);
            unlink(filename.c_str());
            removeFileNode(getReadPolicyIndex(_policy));
            continue;
        }

        // Everything was successful
        close(fd);
        success = true;
        size = (size_t)toRead;
        break;
    }

    return success;
}

void DiskQueue::popFront() {
    if (!_running) {
        return;
    }

    // The lock here is to prevent the writer from catching up with the reader
    const std::lock_guard<RecursiveMutex> lock(_lock);

    if (_fileList.isEmpty()) {
        return; // Nothing available
    }

    auto entry = _fileList.first(); // TODO: Apply policy
    unsigned long fileN = entry->n;
    String filename = _path + String(fileN);

    unlink(filename.c_str());
    removeFileNode(getReadPolicyIndex(_policy));
}

bool DiskQueue::pushBack(const uint8_t* data, size_t size) {
    CHECK_TRUE(_running, false);
    CHECK_TRUE((0 != size), false);
    // A disk limit of zero means that no new items can be enqueued
    CHECK_TRUE((0 < _diskLimit), false);

    // The lock here is to prevent the reader from catching up with the writer
    const std::lock_guard<RecursiveMutex> lock(_lock);

    unsigned long fileN = 0;
    if (!_fileList.isEmpty()) {
        fileN = _fileList.last()->n + 1;
    }

    String filename = _path + String(fileN);

    auto fd = open(filename.c_str(), O_CREAT | O_RDWR | O_APPEND, 0664);
    if (0 > fd) {
        return false;
    }

    do {
        QueueFileHeader fileHeader = { QueueFileMagic, QueueFileVersion1, 0x00 /* no flags */ };
        size_t written = 0;
        auto ret = write(fd, &fileHeader, sizeof(fileHeader));
        if (0 >= ret) {
            break;
        }
        written += (size_t)ret;

        QueueItemHeader itemHeader = { QueueItemMagic, ItemFlagActive, (uint16_t)size };
        ret = write(fd, &itemHeader, sizeof(itemHeader));
        if (0 >= ret) {
            break;
        }
        written += (size_t)ret;

        ret = write(fd, data, size);
        if (0 >= ret) {
            break;
        }
        written += (size_t)ret;

        size_t fileSize = sizeof(fileHeader) + sizeof(itemHeader) + size;
        if ((size_t)written != fileSize) {
            break;
        }

        fsync(fd);
        close(fd);
        addFileNode(fileN, fileSize);

        while (_diskCurrent > _diskLimit) {
            auto index = getWriteOverflowPolicyIndex(_policy);
            auto entry = _fileList.at(index);
            String toDelete = _path + String(entry->n);
            removeFileNode(index);
            unlink(toDelete.c_str());
        }
        return true;
    } while (false);

    close(fd);
    unlink(filename.c_str());
    return false;
}

bool DiskQueue::pushBack(const char* data) {
    auto size = strlen(data);
    return pushBack((uint8_t*)data, size);
}

bool DiskQueue::pushBack(const String& data) {
    return pushBack((uint8_t*)data.c_str(), (size_t)data.length());
}

/**
 * @brief Get list of file numbers that represent disk queue data filenames.
 *
 * @return Vector<unsigned long> An array of the file numbers.
 */
Vector<unsigned long> DiskQueue::list() {
    Vector<unsigned long> fileList;

    for (auto item = _fileList.begin();item != _fileList.end();++item) {
        fileList.append((*item)->n);
    }

    return fileList;
}

bool DiskQueue::dirExists(const char* path) {
    struct stat st = {};
    if (!stat(path, &st)) {
        if (S_ISDIR(st.st_mode)) {
            return true;
        }
    }

    return false;
}

int DiskQueue::sortPartition(Vector<FileEntry*>& array, int begin, int end) {
    FileEntry* last = array[end];
    int pivot = (begin - 1);

    for (int i = begin; i <= (end - 1); ++i) {
        if (array[i]->n <= last->n) {
            std::swap<FileEntry*>(array[++pivot], array[i]);
        }
    }
    std::swap<FileEntry*>(array[pivot + 1], array[end]);

    return (pivot + 1);
}

void DiskQueue::quickSortFiles(Vector<FileEntry*>& array, int begin, int end)
{
    if (array.isEmpty()) {
        // Done
        return;
    }

    Vector<int> stack(end - begin + 1, 0);

    int top = -1;

    stack[++top] = begin;
    stack[++top] = end;

    while (0 <= top) {
        end = stack[top--];
        begin = stack[top--];

        int pivot = sortPartition(array, begin, end);

        if ((pivot - 1) > begin) {
            stack[++top] = begin;
            stack[++top] = pivot - 1;
        }

        if ((pivot + 1) < end) {
            stack[++top] = pivot + 1;
            stack[++top] = end;
        }
    }
}

void DiskQueue::cleanupFiles() {
    if (!_fileList.isEmpty()) {
        for (auto item = _fileList.begin(); _fileList.end() != item; ++item) {
            removeFileNode(*item);
        }
    }

    _fileList.clear();
}

void DiskQueue::unlinkFiles() {
    // The lock here is to prevent the reader and writer from running
    const std::lock_guard<RecursiveMutex> lock(_lock);

    if (!_fileList.isEmpty()) {
        for (auto item = _fileList.begin(); _fileList.end() != item; ++item) {

            unsigned long fileN = (*item)->n;
            String filename = _path + String(fileN);

            auto fd = open(filename.c_str(), O_RDWR, 0664);
            if(0 <= fd) {
                // File open is succesfull so remove file and continue
                unlink(filename.c_str());
                continue;
            }
        }
    }
}

void DiskQueue::cleanup() {
}

DiskQueue::FileEntry* DiskQueue::addFileNode(unsigned long n, size_t size, bool append) {
    FileEntry* entry = new FileEntry;
    if (!entry) {
        return nullptr;
    }
    entry->n = n;
    entry->size = size;

    if (append) {
        _fileList.append(entry);
        _diskCurrent += size;
    }

    return entry;
}

void DiskQueue::removeFileNode(FileEntry* entry) {
    delete entry;
}

void DiskQueue::removeFileNode(int index) {
    if ((0 <= index) && (_fileList.size() > index)) {
        auto entry = _fileList.at(index);
        if (_diskCurrent >= entry->size) {
            _diskCurrent -= entry->size;
        } else {
            // TODO: illegal, assert here?
            _diskCurrent = 0;
        }
        removeFileNode(entry);
        _fileList.removeAt(index);
    }
}

int DiskQueue::getFilenames(const char* path) {
    auto dir = opendir(path);
    if (!dir) {
        return SYSTEM_ERROR_NOT_FOUND;
    }

    struct dirent* ent = nullptr;
    while (nullptr != (ent = readdir(dir))) {
        if (DT_REG != ent->d_type) {
            continue;
        }

        String filename = String(path) + "/" + String(ent->d_name);
        struct stat st = {};
        stat(filename.c_str(), &st);
        char* stop = nullptr;
        unsigned long n = strtoul(ent->d_name, &stop, 10);
        if (strlen(ent->d_name) == (size_t)(stop - ent->d_name)) {
            FileEntry* entry = addFileNode(n, st.st_size);
            CHECK_TRUE(entry, SYSTEM_ERROR_NO_MEMORY);
        }
    }

    quickSortFiles(_fileList, 0, _fileList.size() - 1);

    closedir(dir);

    return SYSTEM_ERROR_NONE;
}

