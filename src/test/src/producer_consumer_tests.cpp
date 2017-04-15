#include <gtest/gtest.h>

#include <util/consumer.hpp>
#include <util/producer.hpp>
#include <util/concurrent_queue.hpp>


namespace
{

// helper class for testing
class SimpleProducer : public Producer<ConcurrentQueue<int>>
{
public:
    void run()
    {
        for (int i = 0; i < 100; ++i)
            for (auto q : queues)
                q->put(std::move(i));
    }
};

}


TEST(producerConsumer, basic)
{
    constexpr int numQueues = 10, numConsumersPerQueue = 10;
    std::vector<ConcurrentQueue<int>> queues(numQueues);
    std::vector<std::thread> consumerThreads;
    for (auto &q : queues)
        for (int i = 0; i < numConsumersPerQueue; ++i)
        {
            consumerThreads.emplace_back([&]()
            {
                Consumer<ConcurrentQueue<int>> consumer(q);
                consumer.run();
            });
        }

    std::thread producerThread([&]()
    {
        SimpleProducer producer;
        for (auto &q : queues)
            producer.addQueue(&q);
        producer.run();
    });

    producerThread.join();
    for (auto &t : consumerThreads)
        t.join();

    for (const auto &q : queues)
        EXPECT_TRUE(q.empty());
}
