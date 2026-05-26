#include "haru25_webgui/server.hpp"
#include "haru25_webgui/haru25_ros2.hpp"
// #include "data.hpp"

std::shared_ptr<Haru25_ros2> haru_ros;

void websocket_session::on_read(beast::error_code ec,
                                std::size_t bytes_transferred) {
    boost::ignore_unused(bytes_transferred);

    // This indicates that the websocket_session was closed
    if (ec == websocket::error::closed) return;

    if (ec) fail(ec, "read");

    //std::cout << "thread: " << boost::this_thread::get_id() << " size: " << bytes_transferred << std::endl;

    ws_.binary(ws_.got_binary());
    // サーバーからデータを受取り、ROS2上のデータを返す
    ws_.async_write(haru_ros->getBuffer(buffer_, bytes_transferred),beast::bind_front_handler(&websocket_session::on_write, shared_from_this()));
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    // ノードを作成
    haru_ros = std::make_shared<Haru25_ros2>();

    // サーバーの設定
    auto const address = net::ip::make_address("0.0.0.0");
    auto const port = static_cast<unsigned short>(25565);
    auto const doc_root = std::make_shared<std::string>(haru_ros->www_path);
    const int threads = 5;

    std::cout << "Start Server! address: " << address << " port: " << port
              << " threads: " << threads << std::endl;

    // The io_context is required for all I/O
    net::io_context ioc{threads};

    // Create and launch a listening port
    std::make_shared<listener>(ioc, tcp::endpoint{address, port}, doc_root)
        ->run();

    // Capture SIGINT and SIGTERM to perform a clean shutdown
    net::signal_set signals(ioc, SIGINT, SIGTERM);
    signals.async_wait([&](beast::error_code const&, int) {
        // Stop the `io_context`. This will cause `run()`
        // to return immediately, eventually destroying the
        // `io_context` and all of the sockets in it.
        ioc.stop();
    });

    // 別スレッドでサーバーを起動
    std::vector<std::thread> v;
    v.reserve(threads-1);
    for (int i=0; i<(threads-1); ++i){
        v.emplace_back([&ioc] { ioc.run(); });
    }

    // ROS2も別スレッドで起動
    v.emplace_back([] { rclcpp::spin(haru_ros); });

    // 1体いる必要あり
    ioc.run();
    // 終了すると、他のスレッドにも終了信号がいく

    // ROS2ノードを終了
    rclcpp::shutdown();

    // 起動中のスレッドが全て終了するまで待つ
    for(auto &t:v){
        t.join();
    }

    // メモリ開放
    haru_ros.reset();

    return EXIT_SUCCESS;
}