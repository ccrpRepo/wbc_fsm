#ifndef READ_TRAJ_H
#define READ_TRAJ_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>

class BinaryArrayReader
{
public:
    // Read array from binary file
    template <typename T>
    static bool readArray(const std::string &filepath, std::vector<T> &data, std::vector<uint32_t> &shape)
    {
        std::ifstream file(filepath, std::ios::binary);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file: " << filepath << std::endl;
            return false;
        }

        // Read and verify magic number
        char magic[4];
        file.read(magic, 4);
        if (std::memcmp(magic, "NPZ\0", 4) != 0)
        {
            std::cerr << "Invalid file format: " << filepath << std::endl;
            return false;
        }

        // Read number of dimensions
        uint32_t ndims;
        file.read(reinterpret_cast<char *>(&ndims), sizeof(uint32_t));

        // Read dimensions
        shape.resize(ndims);
        file.read(reinterpret_cast<char *>(shape.data()), ndims * sizeof(uint32_t));

        // Read data type info
        uint32_t dtype_size;
        file.read(reinterpret_cast<char *>(&dtype_size), sizeof(uint32_t));

        char dtype_code;
        file.read(&dtype_code, 1);

        // Skip reserved bytes
        file.seekg(3, std::ios::cur);

        // Calculate total number of elements
        size_t total_elements = 1;
        for (uint32_t dim : shape)
        {
            total_elements *= dim;
        }

        // Read data
        data.resize(total_elements);
        file.read(reinterpret_cast<char *>(data.data()), total_elements * sizeof(T));

        if (!file.good())
        {
            std::cerr << "Error reading data from file: " << filepath << std::endl;
            return false;
        }

        return true;
    }

    // Print array information
    template <typename T>
    static void printArrayInfo(const std::string &name, const std::vector<T> &data, const std::vector<uint32_t> &shape)
    {
        std::cout << "Array: " << name << std::endl;
        std::cout << "  Shape: (";
        for (size_t i = 0; i < shape.size(); ++i)
        {
            if (i > 0)
                std::cout << ", ";
            std::cout << shape[i];
        }
        std::cout << ")" << std::endl;
        std::cout << "  Elements: " << data.size() << std::endl;
        std::cout << "  Data type size: " << sizeof(T) << " bytes" << std::endl;

        // Print first few elements
        size_t print_count = std::min(static_cast<size_t>(10), data.size());
        std::cout << "  First " << print_count << " elements: [";
        for (size_t i = 0; i < print_count; ++i)
        {
            if (i > 0)
                std::cout << ", ";
            std::cout << data[i];
        }
        std::cout << "]" << std::endl
                  << std::endl;
    }

    static bool readBinFilesFromFolder(
        const std::string &folder_name,
        std::vector<float> &body_ang_vel_w, std::vector<uint32_t> &body_ang_vel_w_shape,
        std::vector<float> &body_lin_vel_w, std::vector<uint32_t> &body_lin_vel_w_shape,
        std::vector<float> &body_pos_w, std::vector<uint32_t> &body_pos_w_shape,
        std::vector<float> &body_quat_w, std::vector<uint32_t> &body_quat_w_shape,
        std::vector<int64_t> &fps, std::vector<uint32_t> &fps_shape,
        std::vector<float> &joint_pos, std::vector<uint32_t> &joint_pos_shape,
        std::vector<float> &joint_vel, std::vector<uint32_t> &joint_vel_shape)
    {
        std::string folder_path = folder_name;

        std::string command = "find " + folder_path + " -name '*.bin' 2>/dev/null";
        FILE *pipe = popen(command.c_str(), "r");
        if (!pipe)
        {
            std::cerr << "Failed to list files in folder: " << folder_path << std::endl;
            return false;
        }

        char buffer[1024];
        std::vector<std::string> bin_files;
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
        {
            std::string filepath(buffer);
            filepath.erase(std::remove(filepath.begin(), filepath.end(), '\n'), filepath.end());
            if (!filepath.empty())
            {
                bin_files.push_back(filepath);
            }
        }
        pclose(pipe);

        if (bin_files.empty())
        {
            std::cout << "No .bin files found in folder: " << folder_path << std::endl;
            return false;
        }
        std::sort(bin_files.begin(), bin_files.end());
        bool all_success = true;
        
        for (const auto &filepath : bin_files)
        { 
            size_t last_slash = filepath.find_last_of('/');
            std::string filename = (last_slash != std::string::npos) ? filepath.substr(last_slash + 1) : filepath;
            std::string name = filename.substr(0, filename.find_last_of('.'));


            bool success = false;

            if (name == "body_ang_vel_w")
            {
                success = readArray(filepath, body_ang_vel_w, body_ang_vel_w_shape);
            }
            else if (name == "body_lin_vel_w")
            {
                success = readArray(filepath, body_lin_vel_w, body_lin_vel_w_shape);
            }
            else if (name == "body_pos_w")
            {
                success = readArray(filepath, body_pos_w, body_pos_w_shape);
            }
            else if (name == "body_quat_w")
            {
                success = readArray(filepath, body_quat_w, body_quat_w_shape);
            }
            else if (name == "fps")
            {
                success = readArray(filepath, fps, fps_shape);
            }
            else if (name == "joint_pos")
            {
                success = readArray(filepath, joint_pos, joint_pos_shape);
            }
            else if (name == "joint_vel")
            {
                success = readArray(filepath, joint_vel, joint_vel_shape);
            }
            else
            {
                std::cerr << "Unknown file: " << filename << std::endl;
            }

            if (!success)
            {
                std::cerr << "Failed to read file: " << filepath << std::endl;
                all_success = false;
            }
        }
                  
        return all_success;
    }
};

#endif // READ_TRAJ_H