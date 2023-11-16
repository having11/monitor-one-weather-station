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

#pragma once

#include "Particle.h"

/**
 * @brief Structure for holding status and diagnostics information
 */
struct DiskQueueStats {
    size_t filesTotal;

};

enum class DiskQueuePolicy {
    FifoDeleteOld,
    FifoDeleteNew,
};

/**
 * @brief The <code>DiskQueue</code> class represents a disk-based queue that
 *
 */

class DiskQueue {

public:
    /**
     * @brief Construct a new DiskQueue object with the given optional file capacity and disk limit
     *
     * @param[in]   diskLimit       Total disk space reserved for queue
     */
    DiskQueue(size_t diskLimit = 0)
    : _diskLimit(diskLimit),
      _diskCurrent(0),
      _policy(DiskQueuePolicy::FifoDeleteOld),
      _running(false) {

    }

    /**
     * @brief Destroy the DiskQueue object
     *
     */
    ~DiskQueue() {
        cleanupFiles();
        cleanup();
    }

    /**
     * @brief Start the disk queue.  This creates the cache queues if they haven't already been
     * created.  The path is checked to exist and created if nonexistant.  This will not create
     * intermediate directories above the path if nested.
     *
     * @param[in]   path            Full directory path for storing the queue on the file system, eg `/usr/my_queue`
     * @param[in]   policy          Queue policy
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_NO_MEMORY
     * @retval SYSTEM_ERROR_FILE
     */
    int start(const char* path, DiskQueuePolicy policy = DiskQueuePolicy::FifoDeleteOld);

    /**
     * @brief Start the disk queue.  This creates the cache queues if they haven't already been
     * created.  The path is checked to exist and created if nonexistant.  This will not create
     * intermediate directories above the path if nested.
     *
     * @param[in]   path            Full directory path for storing the queue on the file system, eg `/usr/my_queue`
     * @param[in]   diskLimit       Total disk space reserved for queue
     * @param[in]   policy          Queue policy
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_NO_MEMORY
     * @retval SYSTEM_ERROR_FILE
     */
    int start(const char* path, size_t diskLimit, DiskQueuePolicy policy = DiskQueuePolicy::FifoDeleteOld) {
        setDiskLimit(diskLimit);
        return start(path, policy);
    }

    /**
     * @brief Stop the disk queue.  Flush both read and write queues to disk in anticipation
     * of service disruption.
     *
     * @retval SYSTEM_ERROR_NONE
     */
    int stop();

    /**
     * @brief Set the disk limit.
     *
     * @param size Size in bytes.
     */
    void setDiskLimit(size_t size);

    /**
     * @brief Get the disk limit in bytes.
     *
     * @return size_t Size in bytes.
     */
    size_t getDiskLimit() const {
        return _diskLimit;
    }

    /**
     * @brief Get the current disk usage in bytes.
     *
     * @return size_t Size in bytes.
     */
    size_t getCurrentDiskUsage() const {
        return _diskCurrent;
    }

    /**
     * @brief Remove front item from read queue if available.
     */
    void popFront(); //TODO: should this method signature be similar to peek_front ?

    /**
     * @brief Inspect item from read queue if available.
     *
     * @param[out]     data     Buffer to copy the data into
     * @param[in,out]  size     [in] maximum buffer size available for copying data into, [out] size written
     * @return true Item has been taken and is in output object
     * @return false No item is available
     */
    bool peekFront(uint8_t* data, size_t& size);

    /**
     * @brief Get size of data from read queue if available.
     *
     * @return Size of data if available; otherwise, zero if empty
     */
    size_t peekFrontSize();

    /**
     * @brief Push item to write queue if space available
     *
     * @param[in]      data     Where to copy data from
     * @param[in]      size     size of the input data
     * @return true Item has been pushed
     * @return false Item has not been pushed
     */
    bool pushBack(const uint8_t* data, size_t size);

    /**
     * @brief Push null terminated character string item to write queue if space available
     *
     * @param[in]      data     Where to copy string data from
     * @return true Item has been pushed
     * @return false Item has not been pushed
     */
    bool pushBack(const char* data);

    /**
     * @brief Push String item to write queue if space available
     *
     * @param[in]      data     Where to copy data from
     * @return true Item has been pushed
     * @return false Item has not been pushed
     */
    bool pushBack(const String& data);

    /**
     * @brief Indicate whether the queue is empty.
     *
     * @return true Queue is empty
     * @return false Queue is not empty
     */
    bool isEmpty() const {
        return _fileList.isEmpty();
    }

    /**
     * @brief Get the size of elements on disk in bytes.
     *
     * @return size_t Number of bytes on disk
     */
    size_t size() const {
        return (size_t)_fileList.size();
    }

    /**
     * @brief Destroy file number array.
     *
     */
    void cleanupFiles();

    /**
     * @brief Unlink/remove files in number array.
     *
     */
    void unlinkFiles();

    /**
     * @brief Destroy read and write cache queues.
     *
     */
    void cleanup();

    /**
     * @brief Get list of file numbers that represent disk queue data filenames.
     *
     * @return Vector<unsigned long> An array of the file numbers.
     */
    Vector<unsigned long> list();

private:
    static constexpr uint8_t QueueFileMagic = 'P';          //< Magic number that must be present at the beginning of each queue file
    static constexpr uint8_t QueueFileVersion1 = 0x01;      //< Current version of the file
    static constexpr uint8_t FileFlagReverse = (1 << 0);    //< Flag to indicate that the queue is to be popped in reverse order

    static constexpr uint8_t QueueItemMagic = 0xf0;         //< Magic number that must be present at the beginning of each queue item
    static constexpr uint8_t ItemFlagActive = (1 << 0);     //< Flag to indicate that the queue item is still active

#pragma pack(push,1)
    struct QueueFileHeader {
        uint8_t magic;          //< Magic number must be 'P'
        uint8_t version;        //< Version of the header structures in this file
        uint8_t flags;          //< Various file-wide flags
    };
    struct QueueItemHeader {
        uint8_t magic;          //< Magic number must be 0xf0
        uint8_t flags;          //< Various item specific flags
        uint16_t length;        //< Length of data immediately following this structure
    };
#pragma pack(pop)

    /**
     * @brief A structure containing the disk based file numbers and filenames.
     *
     */
    struct FileEntry {
        unsigned long n;
        int fd;
        size_t size;
    };

    /**
     * @brief Detect if path exists.
     *
     * @param[in]   path            Full path of file or directory
     * @return true File or directory exists
     * @return false File or directory doesn't exist
     */
    bool dirExists(const char* path);

    /**
     * @brief Quicksort partition for file numbers.
     *
     * @param[in,out]   array       Array of file numbers to partition
     * @param[in]       begin       First index to consider (inclusive)
     * @param[in]       end         Last index to consider (inclusive)
     * @return int The pivot of the partition.
     */
    int sortPartition(Vector<FileEntry*>& array, int begin, int end);

    /**
     * @brief Quicksort algorithm for file numbers.
     *
     * @param[in,out]   array       Array of file numbers to sort
     * @param[in]       begin       First index to consider (inclusive)
     * @param[in]       end         Last index to consider (inclusive)
     */
    void quickSortFiles(Vector<FileEntry*>& array, int begin, int end);

    /**
     * @brief Create (allocate) FileEntry object and initialize it with given
     * file number and file name.  Optionally append it to file list.
     *
     * @param[in]   n               File number, also filename
     * @param[in]   size            File size.
     * @param[in]   append          Append to end of file list.  True to append.  False to not append.
     * @return FileEntry* Allocated FileEntry object pointer.  nullptr if unsuccessful.
     */
    FileEntry* addFileNode(unsigned long n, size_t size, bool append = true);

    /**
     * @brief Destroy FileEntry object.
     *
     * @param[in]   entry           FileEntry object to delete.
     */
    void removeFileNode(FileEntry* entry);

    /**
     * @brief Remove and destroy FileEntry object from file list.
     *
     * @param[in]   index           Index into the file list.
     */
    void removeFileNode(int index);

    enum class ItemState {
        InvalidMagic,
        Active,
        NotActive,
    };

    /**
     * @brief Get the file numbers associated under the given path.
     *
     * @param[in]   path            Full path for search
     * @retval SYSTEM_ERROR_NONE
     * @retval SYSTEM_ERROR_NOT_FOUND
     * @retval SYSTEM_ERROR_NO_MEMORY
     */
    int getFilenames(const char* path);

    /**
     * @brief Get read index
     *
     * @param[in]   policy          Policy to apply for read file deletion
     * @return int Index for read deletion
     */
    int getReadPolicyIndex(DiskQueuePolicy policy);

    /**
     * @brief Get the index to delete
     *
     * @param[in]   policy          Policy to apply for write overflow file deletion
     * @return int Index for read deletion
     */
    int getWriteOverflowPolicyIndex(DiskQueuePolicy policy);

    RecursiveMutex _lock;
    Vector<FileEntry*> _fileList;
    size_t _diskLimit;
    size_t _diskCurrent;
    String _path;
    DiskQueuePolicy _policy;
    bool _running;
};
