#pragma once

#include <algorithm>
#include <boost/asio/bind_executor.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/make_unique.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace beast = boost::beast;          // from <boost/beast.hpp>
namespace http = beast::http;            // from <boost/beast/http.hpp>
namespace websocket = beast::websocket;  // from <boost/beast/websocket.hpp>
namespace net = boost::asio;             // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;        // from <boost/asio/ip/tcp.hpp>

// Return a reasonable mime type based on the extension of a file.
beast::string_view mime_type(beast::string_view);

// Append an HTTP rel-path to a local filesystem path.
// The returned path is normalized for the platform.
std::string path_cat(beast::string_view base, beast::string_view);

// This function produces an HTTP response for the given
// request. The type of the response object depends on the
// contents of the request, so the interface requires the
// caller to pass a generic lambda for receiving the response.
template <class Body, class Allocator, class Send>
void handle_request(beast::string_view, http::request<Body, http::basic_fields<Allocator>>&&, Send&&);

//------------------------------------------------------------------------------

// Report a failure
void fail(beast::error_code ec, char const* what);

// Echoes back all received WebSocket messages
class websocket_session : public std::enable_shared_from_this<websocket_session> {
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;

   public:
    // Take ownership of the socket
    explicit websocket_session(tcp::socket&&);

    // Start the asynchronous accept operation
    template <class Body, class Allocator>
    void do_accept(http::request<Body, http::basic_fields<Allocator>>);

   private:
    void on_accept(beast::error_code);

    void do_read();

    void on_read(beast::error_code, std::size_t);

    void on_write(beast::error_code, std::size_t);
};
//------------------------------------------------------------------------------

// Handles an HTTP server connection
class http_session : public std::enable_shared_from_this<http_session> {
        // This queue is used for HTTP pipelining.
    class queue {
        enum {
            // Maximum number of responses we will queue
            limit = 8
        };

        // The type-erased, saved work item
        struct work {
            virtual ~work() = default;
            virtual void operator()() = 0;
        };

        http_session& self_;
        std::vector<std::unique_ptr<work>> items_;

       public:
        explicit queue(http_session& self) : self_(self) {
            static_assert(limit > 0, "queue limit must be positive");
            items_.reserve(limit);
        }

        // Returns `true` if we have reached the queue limit
        bool is_full() const { return items_.size() >= limit; }

        // Called when a message finishes sending
        // Returns `true` if the caller should initiate a read
        bool on_write() {
            BOOST_ASSERT(!items_.empty());
            auto const was_full = is_full();
            items_.erase(items_.begin());
            if (!items_.empty()) (*items_.front())();
            return was_full;
        }

        // Called by the HTTP handler to send a response.
        template <bool isRequest, class Body, class Fields>
        void operator()(http::message<isRequest, Body, Fields>&& msg) {
            // This holds a work item
            struct work_impl : work {
                http_session& self_;
                http::message<isRequest, Body, Fields> msg_;

                work_impl(http_session& self,
                          http::message<isRequest, Body, Fields>&& msg)
                    : self_(self), msg_(std::move(msg)) {}

                void operator()() {
                    http::async_write(
                        self_.stream_, msg_,
                        beast::bind_front_handler(&http_session::on_write,
                                                  self_.shared_from_this(),
                                                  msg_.need_eof()));
                }
            };

            // Allocate and store the work
            items_.push_back(
                boost::make_unique<work_impl>(self_, std::move(msg)));

            // If there was no previous work, start this one
            if (items_.size() == 1) (*items_.front())();
        }
    };
    // This queue is used for HTTP pipelining.
    beast::tcp_stream stream_;
    beast::flat_buffer buffer_;
    std::shared_ptr<std::string const> doc_root_;
    queue queue_;

    // The parser is stored in an optional container so we can
    // construct it from scratch it at the beginning of each new message.
    boost::optional<http::request_parser<http::string_body>> parser_;

   public:
    // Take ownership of the socket
    http_session(tcp::socket&&, std::shared_ptr<std::string const> const&);

    // Start the session
    void run();

   private:
    void do_read();

    void on_read(beast::error_code, std::size_t);

    void on_write(bool, beast::error_code, std::size_t);

    void do_close();
};

//------------------------------------------------------------------------------

// Accepts incoming connections and launches the sessions
class listener : public std::enable_shared_from_this<listener> {
    net::io_context& ioc_;
    tcp::acceptor acceptor_;
    std::shared_ptr<std::string const> doc_root_;

   public:
    listener(net::io_context&, tcp::endpoint, std::shared_ptr<std::string const> const&);

    void run();

   private:
    void do_accept();

    void on_accept(beast::error_code ec, tcp::socket socket);
};