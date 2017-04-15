#pragma once

#include <atomic>


/// Helper wrapper around atomic bool to cancel multiple tasks simultaneously.
class CancellationToken
{
public:
    CancellationToken()
        : cancel(false)
    {
    }

    void trigger()
    {
        cancel = true;
    }

    bool isTriggered() const
    {
        return cancel.load();
    }

    operator bool() const
    {
        return cancel.load();
    }

private:
    std::atomic_bool cancel;
};
