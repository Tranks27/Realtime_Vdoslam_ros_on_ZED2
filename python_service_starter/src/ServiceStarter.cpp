#include "ServiceStarter.hpp"
#include <signal.h>

#include <stdexcept>

// ServiceStarter::ServiceStarter(const std::string& name, const int* pipes, const int pid) :
//     program_name(name),
//     pid_result(pid),
//     run(true)
// {
//     //this one should be read_fd
//     pipe_py_to_cpp[0] = pipes[0];
//     pipe_py_to_cpp[1] = pipes[1];

// }

ServiceStarter::ServiceStarter(const std::string& _full_path) :
    full_file_path(_full_path),
    run(true),
    status(false)
{

    set_program_name(full_file_path);
    //this one should be read_fd
    // pipe_py_to_cpp[0] = pipes[0];
    // pipe_py_to_cpp[1] = pipes[1];

}


ServiceStarter::~ServiceStarter() {}


void ServiceStarter::run_listener() {
    ROS_INFO_STREAM("Beginning listener thread for python script: " << program_name);
    listener_thread = std::thread(&ServiceStarter::listen_to_program, this);

}

bool ServiceStarter::start_program() {
    // //TODO
        //change this to be the pipecommsmanager array for memory?

        if (::pipe(pipe_py_to_cpp)) {
            ROS_WARN_STREAM("Could not open pipes");
            return false;
        }
        else {
            pid_result = fork();
            status = true;


            if (pid_result == 0) {
                ROS_INFO_STREAM("Setting up env variables for program " << file_name);
                ::close(pipe_py_to_cpp[0]);

                std::ostringstream oss;

                ROS_INFO_STREAM(pipe_py_to_cpp[1]);

                setenv(std::string(program_name + "_PY_WRITE_FD").c_str(), std::to_string(pipe_py_to_cpp[1]).c_str(), 1);
                setenv("PYTHONUNBUFFERED", "true", 1);
                // oss << "python3 -m memory_profiler " << full_file_path;
                oss << "python3 " << full_file_path;


                ::system(oss.str().c_str());
                ::close(pipe_py_to_cpp[1]);
                ros::Duration(3).sleep();
            }
            else if (pid_result < 0) {
                ROS_WARN_STREAM("Program forking failed");
                status = false;
                return false;
            }
            else {
                //is parent
                ::close(pipe_py_to_cpp[1]);
                run_listener();
                return true;
                

            }

                
        }

}

const std::string& ServiceStarter::get_program_name() {
    return program_name;
}

const bool ServiceStarter::get_status() {
    return status;
}

const int ServiceStarter::get_read_fd() {
    return pipe_py_to_cpp[0];
}

void ServiceStarter::listen_to_program() {

    int read_fd = get_read_fd();
    uint32_t message_size;
    std::string python_message;


    while (run) {
        try {

            if (!read_uint32(message_size)) {
                throw std::runtime_error("Could not read uint32");
            }

            if (!read_string(python_message, message_size)) {
                throw std::runtime_error("Could not read message");
            }

            ROS_INFO_STREAM("[" << program_name << "] " << python_message);

        }
        catch(std::exception& e) {
            ROS_WARN_STREAM("Listen to program (" << program_name << ") exception. Closing pipes and exiting");
            shutdown(); 
            return;   
        }
    }


    //doing this is dangerous as we create the memory for pipe_py_to_cpp in a different space and then close it here
    //but cant think of another way of doing this as we need to listen in a thread!
    shutdown();    

}

//https://claytonrichey.com/post/c-cpp-python-pipe/
/* return true if val is set, false for EOF */
bool ServiceStarter::read_uint32(uint32_t &value) {
    unsigned char msgSizeBuf[4];
    unsigned iBuf = 0;
    int read_fd = get_read_fd();

    while (iBuf < sizeof(msgSizeBuf))
    {
        ssize_t rc = ::read(read_fd, msgSizeBuf + iBuf, sizeof(msgSizeBuf) - iBuf);

        if (rc == 0)
        {
            return false;
        }
        else if (rc < 0 )
        {
            ROS_ERROR_STREAM(__func__ << "@" << __LINE__ << ":::Read ERROR");
            return false;
        }
        else
        {
            iBuf += rc;
        }
    }

    value = *(static_cast<uint32_t *>(static_cast<void *>(&msgSizeBuf[0])));
    
    return true;

}
bool ServiceStarter::read_string(std::string& output,uint32_t size) {
    std::vector<char> msgBuf( size + 1 );
    msgBuf[ size ] = '\0';
    unsigned iBuf = 0;

    int read_fd = get_read_fd();

    while (iBuf < size)
    {
        ssize_t rc = ::read(read_fd, &(msgBuf[0]) + iBuf, size - iBuf);

        if ( rc == 0 )
        {
            ROS_WARN_STREAM(__func__ << "@" << __LINE__ << ":::EOF read during message from python script " << program_name);
            return false;
        }
        else if ( rc < 0 )
        {
            ROS_ERROR_STREAM(__func__ << "@" << __LINE__ << ":::Read ERROR during message from python script " << program_name);
            return false;
        }
        else
        {
            iBuf += rc;
        }
    }
    msgBuf.shrink_to_fit();
    std::string s(msgBuf.begin(), msgBuf.end());
    output = s;
    return true;
    // return std::string( &(msgBuf[0]) );
}


void ServiceStarter::set_program_name(std::string& full_path) {

    std::string path = full_path;
    
    const size_t last_slash_idx = path.find_last_of("\\/");
    if (std::string::npos != last_slash_idx) {
        path.erase(0, last_slash_idx + 1);
    }

    file_name = path;
    // Remove extension if present.
    const size_t period_idx = path.rfind('.');
    if (std::string::npos != period_idx)
    {
        path.erase(period_idx);
    }
    ROS_INFO_STREAM("Found file name: " << file_name);
    program_name = path;

    ROS_INFO_STREAM("Found program name: " << program_name);

}


bool ServiceStarter::shutdown() {
    ::close(pipe_py_to_cpp[1]);
    ros::Duration(2).sleep();
    ROS_INFO_STREAM("Killing child process " << pid_result);

    kill(pid_result, SIGKILL);
}

