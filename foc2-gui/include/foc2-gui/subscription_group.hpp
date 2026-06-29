#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>
#include <rclcpp/subscription.hpp>

#include "foc2-gui/im_application.hpp"

template <typename Msg>
class SubscriptionGroup {
public:
    RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionGroup);

    explicit SubscriptionGroup(ImApplication& application)
        : application(application) {}

    void subscribe(const std::string& topic, const rclcpp::QoS& qos) {
        std::scoped_lock lock(mutex);

        if (subscription) {
            subscription.reset();
        }

        subscription = application.create_subscription<Msg>(topic, qos, std::bind(&SubscriptionGroup::onMessage, this, std::placeholders::_1));
    }

    template <typename Owner, typename Callback>
    void addCallback(const std::shared_ptr<Owner>& owner, Callback&& callback) {
        addCallbackImpl(std::weak_ptr<void>(owner), std::function < void(std::shared_ptr<Msg>) > (std::forward<Callback>(callback)));
    }

    template <typename Callback>
    void addCallback(std::weak_ptr<void> owner, Callback&& callback) {
        addCallbackImpl(std::move(owner), std::function < void(std::shared_ptr<Msg>) > (std::forward<Callback>(callback)));
    }

    std::shared_ptr<rclcpp::Subscription<Msg>> getSubscription() {
        return subscription;
    }

private:
    struct CallbackHolder {
        std::weak_ptr<void> owner;
        std::function<void(std::shared_ptr<Msg>)> callback;

        RCLCPP_SMART_PTR_DEFINITIONS(CallbackHolder);
    };

    void addCallbackImpl(std::weak_ptr<void> owner, std::function<void(std::shared_ptr<Msg>)> callback) {
        std::scoped_lock lock(mutex);
        callbacks.push_back(CallbackHolder::make_shared(std::move(owner), std::move(callback)));
    }

    void onMessage(const std::shared_ptr<Msg> msg) {
        std::vector<typename CallbackHolder::SharedPtr> callbacks_copy;

        // copy callbacks to avoid any race conditions where the callbacks vector is mutated as we iterate over them
        {
            std::scoped_lock lock(mutex);

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
    std::shared_ptr<rclcpp::Subscription<Msg>> subscription;

    std::mutex mutex;
    std::vector<typename CallbackHolder::SharedPtr> callbacks;
};
