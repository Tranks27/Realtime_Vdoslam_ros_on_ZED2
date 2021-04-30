#include "vdo_slam/visualizer/colour.h"
#include "vdo_slam/visualizer/colour-map.h"
#include "vdo_slam/visualizer/semantics_loader.h"
#include "vdo_slam/Macros.h"

#include <opencv2/viz/types.hpp>
#include <fstream>


namespace VDO_SLAM {

    cv::RNG HashableColor::rng(1234);


    HashableColor::HashableColor(const cv::viz::Color& color) 
        :   cv::viz::Color(color)
            {
                //opencv colors are saved in BGR form
                r = color[2];
                g = color[1];
                b = color[0];
            }

    HashableColor::HashableColor()
        :   cv::viz::Color(),
            r(-1), g(-1), b(-1) {}

    HashableColor::HashableColor(uint8_t r_, uint8_t g_, uint8_t b_)
        :   Color(b_, g_, r_),
            r(r_), g(g_), b(b_) {}

    bool HashableColor::operator==(const HashableColor& other) const {
        return (r == other.r && g == other.g && b == other.b);
    }

    bool HashableColor::equal(const HashableColor& color) const {
        return r == color.r && g == color.g && b == color.b;
    }

    bool HashableColor::equal(const cv::viz::Color& color) const {
        HashableColor other_color(color);
        return this->equal(other_color);
    }

    inline bool HashableColor::is_valid() {
        return !(r == -1 || g == -1 || b == -1);
    }

    std::ostream &operator << (std::ostream& output, const HashableColor& color) {
        output << "r: " << color.r << " g: " << color.g << " b: " << color.b;
        return output;
    }

    void HashableColor::set_random_seed(const int seed) {
        HashableColor::rng = cv::RNG(seed);
    }

    HashableColor HashableColor::get_random_color() {
        cv::Scalar color = cv::Scalar(HashableColor::rng.uniform(0,255), 
                                      HashableColor::rng.uniform(0, 255), 
                                      HashableColor::rng.uniform(0, 255));
        return HashableColor(color);
    }


    ColourManager::ColourManager(const std::string& filename)
        :   label_to_color(), tracking_id_to_color(), classes() {
        std::ifstream file(filename.c_str());
        size_t row_number = 1;
        for (CSVIterator loop(file); loop != CSVIterator(); ++loop) {
            // We expect the CSV to have header:
            // 0   , 1
            // id, class
            std::string class_label = (*loop)[1];
            classes.push_back(class_label);
            row_number++;
        }

        size_t n_classes = number_of_classes();
        int step_size = 10;

        //this assumes we have less classes than colours in our palette
        int class_index = 0;
        for(size_t i = 0; i < n_classes; i++) {
            int index = step_size * i;
            if (index > MAX_COLOUR_PALETTE) {
                index = index % MAX_COLOUR_PALETTE;
            }
            uint8_t r = (colmap[index][0] * 255);
            uint8_t g = (colmap[index][1] * 255);
            uint8_t b = (colmap[index][2] * 255);
            HashableColor color(r, g, b);

            // if (color.equal(HashableColor::black())) {
            //     uint8_t r = (colmap[index][0] * 255);
            //     uint8_t g = (colmap[index][1] * 255);
            //     uint8_t b = (colmap[index][2] * 255);
            //     HashableColor color(r, g, b);

            // }

            tracking_id_to_color[i] =  color;
            label_to_color[classes[i]] = color;
            // class_index++;

            //for labels
            // if (i % step_size == 0 && class_index < n_classes) {

            // }
        }

    }

    HashableColor ColourManager::get_colour_for_class_label(const std::string& label) {
        return label_to_color[label];
    }
    HashableColor ColourManager::get_colour_for_tracking_id(const int tracking_id) {
        size_t n_classes = number_of_classes();
        int track = tracking_id;
        if (track > n_classes) {
            track = track % n_classes;
        }

        return tracking_id_to_color[track];        
    }


}