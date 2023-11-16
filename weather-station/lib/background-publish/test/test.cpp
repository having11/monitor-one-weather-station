#include "BackgroundPublish.h"
#include <thread>

#define CATCH_CONFIG_MAIN
#include "catch.h"

std::string str = "Publish This";
int high_cb_counter;
int low_cb_counter;
particle::Error status_returned;

void priority_high_cb(particle::Error status,
    const char *event_name,
    const char *event_data) {
    status_returned = status;
    high_cb_counter++;
}

void priority_low_cb(particle::Error status,
    const char *event_name,
    const char *event_data) {
    status_returned = status;
    low_cb_counter++;
}

void priority_high_cb2(particle::Error status,
    const char *event_name,
    const char *event_data,
    const int context) {
    status_returned = status;
    high_cb_counter++;
}

void priority_low_cb2(particle::Error status,
    const char *event_name,
    const char *event_data,
    const int context) {
    status_returned = status;
    low_cb_counter++;
}

class TestBackgroundPublish : public BackgroundPublish<> {
public:
    void processOnce();
};

void TestBackgroundPublish::processOnce()
{
    constexpr std::size_t burst_rate {2u}; // allowable burst rate (Hz), Device OS allows up to 4/s
    constexpr system_tick_t process_interval {1000u};

    // Publish time of the last (burst_rate) sends in a circular buffer
    static system_tick_t publish_t[burst_rate] {};
    static std::size_t i {}; // publish time of the previous (burst_rate)th send

    auto now {millis()};
    if(now - publish_t[i] >= process_interval) {
        for(auto &queue : _queues) {
            if(!queue.empty()) {
                publish_t[i] = now;
                i = (i + 1) % burst_rate;
                // Copy the event and pop so the publish is done without holding the mutex
                publish_event_t event {queue.front()};
                queue.pop();
                process_publish(event);
                break;
            }
        }
    }
}

TEST_CASE("Test Background Publish") {
    TestBackgroundPublish publisher;

    publisher.start();

    for(int i = 0; i < 8; i++) {
        publisher.publish<int>("TEST_PUB_HIGH",
                            str.c_str(),
                            PRIVATE,
                            1,
                            priority_high_cb2, 1);
    }
    REQUIRE(publisher.publish<int>("TEST_PUB_HIGH",
                        str.c_str(), 
                        PRIVATE,
                        2, 
                        priority_high_cb2, 1) == false);

    // CANCELLED, run cleanup()
    high_cb_counter = 0;
    low_cb_counter = 0;
    Particle.state_output.err = particle::Error::NONE;
    Particle.state_output.isDoneReturn = true;

    publisher.cleanup();
    REQUIRE(high_cb_counter == 8);
    REQUIRE(status_returned == particle::Error::CANCELLED);
    high_cb_counter = 0;
    status_returned = particle::Error::UNKNOWN;

    // FAIL, Not enough levels/queues
    REQUIRE(publisher.publish("TEST_PUB_HIGH",
                        str.c_str(), 
                        PRIVATE,
                        2, 
                        priority_high_cb ) == false);
    REQUIRE(high_cb_counter == 1);
    REQUIRE(status_returned == particle::Error::INVALID_ARGUMENT);
    
    // PASS, enough levels/queues
    REQUIRE(publisher.publish("TEST_PUB_HIGH",
                        str.c_str(), 
                        PRIVATE,
                        1,
                        priority_low_cb ) == true);
    REQUIRE(publisher.publish("TEST_PUB_HIGH",
                        str.c_str(),
                        PRIVATE,
                        1,
                        priority_low_cb ) == true);
    System.inc(1000); // increase the tick by one second to allow processOnce to process
    // burst process two messages at t = 1000
    publisher.processOnce();
    publisher.processOnce();
    REQUIRE(low_cb_counter == 2);
    REQUIRE(status_returned == particle::Error::NONE);

    // Fail, not enough time passed between processing publishes
    // Then PASS once once time has passed.
    high_cb_counter = 0;
    low_cb_counter = 0;
    REQUIRE(publisher.publish("TEST_PUB_HIGH",
                        str.c_str(), 
                        PRIVATE,
                        0, 
                        priority_high_cb ) == true);
    System.inc(500); // not enough delay to process
    publisher.processOnce(); //run to clear off the queues
    REQUIRE(high_cb_counter == 0);
    REQUIRE(low_cb_counter == 0);

    System.inc(500); //increase the tick by one second to allow processOnce to process
    publisher.processOnce(); //run to clear off the queues
    REQUIRE(high_cb_counter == 1);
    REQUIRE(low_cb_counter == 0);
    REQUIRE(status_returned == particle::Error::NONE);
    status_returned = particle::Error::UNKNOWN;

    // LIMIT_EXCEEDED, run processOnce and fail on is.Succeeded()
    high_cb_counter = 0;
    low_cb_counter = 0;
    status_returned = particle::Error::UNKNOWN;
    Particle.state_output.err = particle::Error::LIMIT_EXCEEDED;
    publisher.publish("TEST_PUB_HIGH",
                        str.c_str(), 
                        PRIVATE,
                        0, 
                        priority_high_cb);
    System.inc(1000); //increase the tick by one second to allow processOnce to process
    publisher.processOnce();
    REQUIRE(high_cb_counter == 1);
    REQUIRE(low_cb_counter == 0);
    REQUIRE(status_returned == particle::Error::LIMIT_EXCEEDED);
    status_returned = particle::Error::UNKNOWN;

    //NONE, run processOnce and pass on is.Succeeded()
    high_cb_counter = 0;
    low_cb_counter = 0;
    Particle.state_output.isDoneReturn = true;
    Particle.state_output.err = particle::Error::NONE;
    publisher.publish("TEST_PUB_HIGH",
                        str.c_str(), 
                        PRIVATE,
                        0, 
                        priority_high_cb);
    publisher.publish("TEST_PUB_HIGH",
                        str.c_str(), 
                        PRIVATE,
                        0, 
                        priority_high_cb);
    publisher.publish("TEST_PUB_HIGH",
                        str.c_str(), 
                        PRIVATE,
                        0, 
                        priority_high_cb);

    System.inc(1000); //increase the tick by one second to allow processOnce to process
    publisher.processOnce();
    REQUIRE(high_cb_counter == 1);
    REQUIRE(low_cb_counter == 0);
    REQUIRE(status_returned.type() == particle::Error::NONE);
    status_returned = particle::Error::UNKNOWN; //clearout to something

    System.inc(1000); //increase the tick by one second to allow processOnce to process
    publisher.processOnce();
    REQUIRE(high_cb_counter == 2);
    REQUIRE(low_cb_counter == 0);
    REQUIRE(status_returned == particle::Error::NONE);
    status_returned = particle::Error::UNKNOWN; //clearout to something

    System.inc(1000); //increase the tick by one second to allow processOnce to process
    publisher.processOnce();
    REQUIRE(high_cb_counter == 3);
    REQUIRE(low_cb_counter == 0);
    REQUIRE(status_returned == particle::Error::NONE);
    status_returned = particle::Error::UNKNOWN; //clearout to something

    //NONE, publish from high and low priority queues
    high_cb_counter = 0;
    low_cb_counter = 0;
    Particle.state_output.err = particle::Error::NONE;
    publisher.publish("TEST_PUB_LOW",
                        str.c_str(), 
                        PRIVATE,
                        1, 
                        priority_low_cb);
    publisher.publish("TEST_PUB_LOW",
                        str.c_str(), 
                        PRIVATE,
                        1, 
                        priority_low_cb);
    publisher.publish("TEST_PUB_LOW",
                        str.c_str(), 
                        PRIVATE,
                        1, 
                        priority_low_cb);
    publisher.publish("TEST_PUB_HIGH",
                        str.c_str(), 
                        PRIVATE,
                        0, 
                        priority_high_cb);
    publisher.publish("TEST_PUB_HIGH",
                        str.c_str(), 
                        PRIVATE,
                        0, 
                        priority_high_cb);
    publisher.publish("TEST_PUB_HIGH",
                        str.c_str(), 
                        PRIVATE,
                        0, 
                        priority_high_cb);

    System.inc(1000); //increase the tick by one second to allow processOnce to process
    publisher.processOnce();
    REQUIRE(low_cb_counter == 0);
    REQUIRE(high_cb_counter == 1);
    REQUIRE(status_returned == particle::Error::NONE);
    status_returned = particle::Error::UNKNOWN; //clear out to something

    System.inc(1000); //increase the tick by one second to allow processOnce to process
    publisher.processOnce();
    REQUIRE(low_cb_counter == 0);
    REQUIRE(high_cb_counter == 2);
    REQUIRE(status_returned == particle::Error::NONE);
    status_returned = particle::Error::UNKNOWN; //clear out to something

    System.inc(1000); //increase the tick by one second to allow processOnce to process
    publisher.processOnce();
    REQUIRE(low_cb_counter == 0);
    REQUIRE(high_cb_counter == 3);
    REQUIRE(status_returned == particle::Error::NONE);
    status_returned = particle::Error::UNKNOWN; //clear out to something

    System.inc(1000); //increase the tick by one second to allow processOnce to process
    publisher.processOnce();
    REQUIRE(low_cb_counter == 1);
    REQUIRE(high_cb_counter == 3);
    REQUIRE(status_returned == particle::Error::NONE);
    status_returned = particle::Error::UNKNOWN; //clear out to something

    System.inc(1000); //increase the tick by one second to allow processOnce to process
    publisher.processOnce();
    REQUIRE(low_cb_counter == 2);
    REQUIRE(high_cb_counter == 3);
    REQUIRE(status_returned == particle::Error::NONE);
    status_returned = particle::Error::UNKNOWN;

    publisher.processOnce();
    REQUIRE(status_returned == particle::Error::NONE);  // burst send
    REQUIRE(low_cb_counter == 3);
    REQUIRE(high_cb_counter == 3);
}
