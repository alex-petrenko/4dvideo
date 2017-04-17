#pragma once

#include <atomic>


class AppState
{
public:
    static AppState & instance();

    void startGrabbing();
    bool isGrabbingStarted() const;

    void stopGrabbing();
    bool isGrabbingStopped();

private:
    AppState() = default;
    AppState(const AppState &) = delete;
    void operator=(const AppState &) = delete;

private:
    std::atomic_bool start = false, stop = false;
};

AppState & appState();
