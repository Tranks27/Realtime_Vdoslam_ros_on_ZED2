#include <ros/ros.h>

#include <iostream>
#include <thread>

class ServiceStarter {

    public:
        ServiceStarter(const std::string& _full_path);
        ~ServiceStarter();

        bool shutdown();

        const std::string& get_program_name();
        const bool get_status();
        bool start_program();

    private:
        // int pipe_cpp_to_py[2];
        int pipe_py_to_cpp[2];
        std::string full_file_path;

        //program name with .py
        std::string file_name;
        std::string program_name;

        pid_t pid_result;
        bool status;
        bool run;

        std::thread listener_thread;

        //set up stuff to read msgs across pipes
        bool read_uint32(uint32_t &value);
        bool read_string(std::string& output,uint32_t size);

        void run_listener();
        void listen_to_program();
        void set_program_name(std::string& full_path);

        const int get_read_fd();


    
};