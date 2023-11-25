#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <fstream>
#include <iostream>
#include <set>
#include <streambuf>
#include <string>

/* This class was adapted from the WebSocket++ telemetry_server example:
https://github.com/zaphoyd/websocketpp/tree/master/examples/telemetry_server */

class telemetry_server {
public:
    typedef websocketpp::connection_hdl connection_hdl;
    typedef websocketpp::server<websocketpp::config::asio> server;

    telemetry_server() {
        // set up access channels to only log interesting things
        m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
        m_endpoint.set_access_channels(websocketpp::log::alevel::access_core);
        m_endpoint.set_access_channels(websocketpp::log::alevel::app);

        // Initialize the Asio transport policy
        m_endpoint.init_asio();

        // Bind the handlers we are using
        using websocketpp::lib::placeholders::_1;
        using websocketpp::lib::bind;
        m_endpoint.set_open_handler(bind(&telemetry_server::on_open,this,_1));
        m_endpoint.set_close_handler(bind(&telemetry_server::on_close,this,_1));
        // m_endpoint.set_http_handler(bind(&telemetry_server::on_http,this,_1));
    }

    void run(uint16_t port) {
        std::stringstream ss;
        ss << "Running telemetry server on port "<< port;
        m_endpoint.get_alog().write(websocketpp::log::alevel::app,ss.str());
        
        // listen on specified port
        m_endpoint.listen(port);

        // Start the server accept loop
        m_endpoint.start_accept();

        // Start the ASIO io_service run loop
        try {
            m_endpoint.run();
        } catch (websocketpp::exception const & e) {
            std::cout << e.what() << std::endl;
        }
    }

    void send_data(std::string data) {
        con_list::iterator it;
        for (it = m_connections.begin(); it != m_connections.end(); ++it) {
            m_endpoint.send(*it,data,websocketpp::frame::opcode::text);
        }
    }

    void on_open(connection_hdl hdl) {
        m_connections.insert(hdl);
    }

    void on_close(connection_hdl hdl) {
        m_connections.erase(hdl);
    }
private:
    typedef std::set<connection_hdl,std::owner_less<connection_hdl>> con_list;
    
    server m_endpoint;
    con_list m_connections;
};