#include <ros/ros.h>
#include <ros/package.h>
#include <crow.h>
#include <unordered_set>
#include <mutex>

std::string get_mime_type(const std::string &path)
{
    std::size_t last_dot = path.find_last_of(".");
    std::string extension = path.substr(last_dot + 1);
    if (extension != "")
    {
        std::string mimeType = crow::mime_types.at(extension);
        if (mimeType != "")
            return mimeType;
        else
            return "text/plain";
    }
    return "";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_gui");
    ros::NodeHandle nh;
    ROS_INFO("Starting the Robot GUI..");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    std::string host;
    if (ros::param::has("~gui_host"))
        ros::param::get("~gui_host", host);
    else
        host = "127.0.0.1";
    int port;
    if (ros::param::has("~gui_port"))
        ros::param::get("~gui_port", port);
    else
        port = 8080;

    std::mutex mtx;
    std::unordered_set<crow::websocket::connection *> users;
    const std::string templates_path = ros::package::getPath("robot_gui") + "/templates/";
    const std::string static_path = ros::package::getPath("robot_gui") + "/static/";

    crow::SimpleApp app;
    CROW_ROUTE(app, "/")
    ([templates_path]()
     {
            crow::mustache::context ctx;
            ctx["title"] = "Ohmni";
            crow::mustache::set_base(templates_path);
            const auto test = crow::mustache::load("index.html").render(ctx);
            return crow::mustache::load("index.html").render(ctx); });

    CROW_ROUTE(app, "/static/<string>")
    ([static_path](crow::response &res, std::string path)
     {
            std::ifstream ifl(static_path + path);
            std::ostringstream buffer;
            buffer << ifl.rdbuf();
            res.body = buffer.str();
            res.add_header("Content-length", std::to_string(res.body.size()));
            std::string mimeType = get_mime_type(path);
            if (mimeType != "")
                res.add_header("Content-Type", mimeType);
            res.end(); });

    CROW_ROUTE(app, "/static/faces/<string>")
    ([static_path](crow::response &res, std::string path)
     {
            std::ifstream ifl(static_path + "faces/" + path);
            std::ostringstream buffer;
            buffer << ifl.rdbuf();
            res.body = buffer.str();
            res.add_header("Content-length", std::to_string(res.body.size()));
            std::string mimeType = get_mime_type(path);
            if (mimeType != "")
                res.add_header("Content-Type", mimeType);
            res.end(); });

    CROW_ROUTE(app, "/solver")
        .websocket()
        .onopen([&](crow::websocket::connection &conn)
                { std::lock_guard<std::mutex> _(mtx);
                users.insert(&conn); })
        .onclose([&](crow::websocket::connection &conn, const std::string &reason)
                 { std::lock_guard<std::mutex> _(mtx);
                users.erase(&conn); });

    ROS_INFO("Starting GUI Server on host %s with port %d..", host.c_str(), port);
    auto srv_st = std::async(std::launch::async, [&]
                             { app.bindaddr(host).port(port).run(); });
    app.wait_for_server_start();
    ROS_INFO("GUI Server is now running..");

    bool is_happy = true;

    ros::Rate loop_rate(0.2);
    while (ros::ok())
    {
        ros::spinOnce();
        std::string msg = crow::json::wvalue({{"type", "show_face"}, {"facial_expression", is_happy ? "happy" : "idle"}}).dump();
        for (const auto &conn : users)
            conn->send_text(msg);
        is_happy = !is_happy;
        loop_rate.sleep();
    }
}