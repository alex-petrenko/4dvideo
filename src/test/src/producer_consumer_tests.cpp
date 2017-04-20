#include <gtest/gtest.h>

#include <util/consumer.hpp>
#include <util/producer.hpp>
#include <util/concurrent_queue.hpp>
#include <util/cancellation_token.hpp>


namespace
{

// helper class for testing
class SimpleProducer : public Producer<ConcurrentQueue<int>>
{
public:
    SimpleProducer(const CancellationToken &cancel)
        : Producer(cancel)
    {
    }

    void run()
    {
        for (int i = 0; i < 10000; ++i)
            for (auto q : queues)
                q->put(i);
    }
};

}


TEST(producerConsumer, basic)
{
    constexpr int numQueues = 10, numConsumersPerQueue = 10;
    std::vector<ConcurrentQueue<int>> queues(numQueues);
    std::vector<std::thread> consumerThreads;

    CancellationToken cancel;

    for (auto &q : queues)
        for (int i = 0; i < numConsumersPerQueue; ++i)
        {
            consumerThreads.emplace_back([&]()
            {
                Consumer<ConcurrentQueue<int>> consumer(q, cancel);
                consumer.run();
            });
        }

    std::thread producerThread([&]()
    {
        SimpleProducer producer(cancel);
        for (auto &q : queues)
            producer.addQueue(&q);
        producer.run();
    });

    producerThread.join();

    cancel.trigger();

    for (auto &t : consumerThreads)
        t.join();

    for (const auto &q : queues)
        EXPECT_TRUE(q.empty());
}
