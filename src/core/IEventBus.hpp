#pragma once

#include <functional>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace nbody::core {

template<typename EventType>
using EventHandler = std::function<void(const EventType&)>;

class IEventBus {
public:
    virtual ~IEventBus() = default;

    template<typename EventType>
    void publish(const EventType& event) {
        publishImpl(std::type_index(typeid(EventType)), &event);
    }

    template<typename EventType>
    void subscribe(EventHandler<EventType> handler) {
        subscribeImpl(std::type_index(typeid(EventType)), 
                     [handler = std::move(handler)](const void* event) {
                         handler(*static_cast<const EventType*>(event));
                     });
    }

protected:
    virtual void publishImpl(std::type_index type, const void* event) = 0;
    virtual void subscribeImpl(std::type_index type, std::function<void(const void*)> handler) = 0;
};

class EventBus : public IEventBus {
public:
    static std::shared_ptr<EventBus> create() {
        return std::shared_ptr<EventBus>(new EventBus());
    }

protected:
    void publishImpl(std::type_index type, const void* event) override {
        auto it = handlers_.find(type);
        if (it != handlers_.end()) {
            for (const auto& handler : it->second) {
                handler(event);
            }
        }
    }

    void subscribeImpl(std::type_index type, std::function<void(const void*)> handler) override {
        handlers_[type].push_back(std::move(handler));
    }

private:
    EventBus() = default;
    std::unordered_map<std::type_index, std::vector<std::function<void(const void*)>>> handlers_;
};

}  // namespace nbody::core