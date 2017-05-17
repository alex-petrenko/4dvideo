#pragma once

#include <3dvideo/mesh_frame.hpp>


constexpr int targetScreenWidth = 640;

class Player : public MeshFrameConsumer
{
public:
    /// Private implementation to hide some OpenGL headers.
    class PlayerImpl;
    friend class PlayerImpl;

public:
    Player(MeshFrameQueue &q, CancellationToken &cancellationToken);
    virtual ~Player();

    virtual void init();
    virtual void run();

private:
    std::unique_ptr<PlayerImpl> data;
};
