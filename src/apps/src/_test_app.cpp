#include <chrono>
#include <thread>

#include <util/tiny_logger.hpp>

#include <3dvideo/dataset_reader.hpp>
#include <3dvideo/dataset_writer.hpp>
#include <3dvideo/data_visualizer.hpp>


using namespace std::chrono_literals;

#include <thread>
#include <iostream>

class ConThread {
public:
    ConThread()
    {
        std::cout << __FUNCTION__ << std::endl;
    }
    ~ConThread()
    {
        std::cout << __FUNCTION__ << std::endl;
        if (t_.joinable()) {
            std::cout << "Joining thread.." << std::endl;
            t_.join();//avoid a crash because std::thread will terminate the app if the thread is still running in it's destructor
            std::cout << "Joined thread.." << std::endl;
        }
    }

    void start()
    {
        t_ = std::thread(doWorkInternal, this);
    }

    std::thread& get() { return t_; };
protected:
    virtual void doWork() = 0;
private:
    static void doWorkInternal(ConThread* t)
    {
        try {
            t->doWork();
        }
        catch (...)
        {
        };

    }
    std::thread t_;
};

class MyConThread : public ConThread
{
public:
    MyConThread()
    {
        std::cout << __FUNCTION__ << std::endl;
    }

    long i = 0;
protected:
    void doWork() override
    {
        for (long j = 0; j<1000; j++)
        {
            ++i;
        }
        std::cout << __FUNCTION__ << i << std::endl;
    }
};

int main()
{
    std::cout << __FUNCTION__ << std::endl;
    MyConThread mct; //<== crashes when being destroyed because thread calls t->doWork ()
    mct.start();

    std::this_thread::sleep_for(1s);

    return 0;
}
