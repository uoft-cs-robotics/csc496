#pragma once

#include "commons.h"
#include "messages.h"

namespace teleop
{

    template<typename T>
    class msqueue 
    {
        public : 
        
            msqueue() = default;
            msqueue(const msqueue<T>&) = delete;
            virtual ~msqueue() { clear(); }

        
        public: 

            const T& front()
            {
                if (doMux) std::scoped_lock lock(muxQueue);
                return deqQueue.front();
            }

            const T& back()
            {
                if (doMux) std::scoped_lock lock(muxQueue);
                return deqQueue.back();
            }

            void push_front(const T& item)
            {
                if (doMux) std::scoped_lock lock(muxQueue);
                deqQueue.emplace_front(std::move(item));
            }

            void push_back(const T& item)
            {
                if (doMux) std::scoped_lock lock(muxQueue);
                deqQueue.emplace_back(std::move(item));
            }

            bool empty()
            {
                if (doMux) std::scoped_lock lock(muxQueue);
                return deqQueue.empty();
            }

            std::size_t count()
            {
                if (doMux) std::scoped_lock lock(muxQueue);
                return deqQueue.size();
            }

            void clear()
            {
                if (doMux) std::scoped_lock lock(muxQueue);
                deqQueue.clear();
            }

            T pop_front()
            {
                if (doMux) std::scoped_lock lock(muxQueue);
                auto t = std::move(deqQueue.front());
                if (count() > 0)
                {
                    deqQueue.pop_front();
                }

                return t;
            }

            T pop_back()
            {
                if (doMux) std::scoped_lock lock(muxQueue);
                auto t = std::move(deqQueue.back());
                if (count() > 0)
                {
                    deqQueue.pop_front();
                }
                deqQueue.pop_back();
                return t;
            }


        private : 

            bool doMux = true;
            std::mutex muxQueue;
            std::deque<T> deqQueue;

            bool debug = true;

    };

} // teleop
