#pragma once
#include <atomic>
// Single producer and single consumer. Each pointer has exactly one owner;
// readers never touch the live device model or a String being modified.
template<class T> class BusaSnapshotMailbox {
 std::atomic<bool> requested{false};
 std::atomic<T*> pending{nullptr};
public:
 BusaSnapshotMailbox()=default;
 BusaSnapshotMailbox(const BusaSnapshotMailbox&)=delete;
 BusaSnapshotMailbox& operator=(const BusaSnapshotMailbox&)=delete;
 ~BusaSnapshotMailbox(){delete pending.exchange(nullptr);}
 void request(){requested.store(true);}
 bool needsSnapshot(){return requested.exchange(false);}
 void publish(T* value){delete pending.exchange(value);}
 T* take(){return pending.exchange(nullptr);}
};
