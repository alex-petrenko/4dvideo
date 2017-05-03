#pragma once

#include <3dvideo/frame.hpp>


class Player : public FrameConsumer
{
public:
    /// Private implementation to hide some OpenGL headers.
    class PlayerImpl;
    friend class PlayerImpl;

public:
    Player(FrameQueue &q, CancellationToken &cancellationToken);
    virtual ~Player();

    virtual void init();
    virtual void run();

private:
    std::unique_ptr<PlayerImpl> data;
};
