#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>
#include "foc2-gui/im_application.hpp"

template <typename Msg>
class SubscriptionGroup {
public:
    RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionGroup);

    explicit SubscriptionGroup(ImApplication& application) : application(application) {}

    void subscribe(const std::string& topic, const rclcpp::QoS& qos) {
        std::lock_guard lock(mutex);
        if (subscription) {
            RCLCPP_WARN(application.get_logger(), "Tried to subscribe to multiple times to subscription group. Topic: %s", topic.data());
            return;
        }

        subscription = application.create_subscription<Msg>(topic, qos, std::bind(&SubscriptionGroup::onMessageShared, this, std::placeholders::_1));
    }

    template <typename Owner, typename Callback>
    void addCallback(const std::shared_ptr<Owner>& owner, Callback&& callback) {
        addCallbackImpl(std::weak_ptr<void>(owner), std::function<void(typename Msg::SharedPtr)>(std::forward<Callback>(callback)));
    }

    template <typename Callback>
    void addCallback(std::weak_ptr<void> owner, Callback&& callback) {
        addCallbackImpl(std::move(owner), std::function<void(typename Msg::SharedPtr)>(std::forward<Callback>(callback)));
    }

private:
    struct CallbackHolder {
        std::weak_ptr<void> owner;
        std::function<void(typename Msg::SharedPtr)> callback;

        RCLCPP_SMART_PTR_DEFINITIONS(CallbackHolder);
    };

    void addCallbackImpl(std::weak_ptr<void> owner, std::function<void(typename Msg::SharedPtr)> callback) {
        std::lock_guard lock(mutex);
        callbacks.push_back(CallbackHolder::make_shared(std::move(owner), std::move(callback)));
    }

    void onMessageShared(const Msg::ConstSharedPtr& msg) {
        std::vector<typename CallbackHolder::SharedPtr> callbacks_copy;

        // copy callbacks to avoid any race conditions where the callbacks vector is mutated as we iterate over them
        {
            std::lock_guard lock(mutex);

            // delete any expired callbacks
            std::erase_if(callbacks, [](auto& sp) {
                return !sp || sp->owner.expired();
            });

            callbacks_copy = callbacks;
        }

        for (const auto& callback : callbacks_copy) {
            if (const auto owner = callback->owner.lock(); owner && callback->callback)
                callback->callback(msg);
        }
    }

    ImApplication& application;
    rclcpp::Subscription<Msg>::SharedPtr subscription;

    std::mutex mutex;
    std::vector<typename CallbackHolder::SharedPtr> callbacks;
};
