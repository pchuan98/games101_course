#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

std::string converter_to_str(float value, int precision = 2)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

/**
 * @brief Get the view matrix object
 *
 * @param eye_pos
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

/**
 * @brief Get the model matrix object
 * 逐个元素地构建模型变换矩阵并返回该矩阵。
 * 在此函数中，你只需要实现三维中绕 z 轴旋转的变换矩阵，而不用处理平移与缩放。
 * @param rotation_angle
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    /**
     * | cosa -sina 0 0 |
     * | sina cosa  0 0 |
     * | 0    0     1 0 |
     * | 0    0     0 1 |
     */

    model(0, 0) = (float)cos(rotation_angle / 180.0 * MY_PI);
    model(0, 1) = (float)-sin(rotation_angle / 180.0 * MY_PI);
    model(1, 0) = (float)sin(rotation_angle / 180.0 * MY_PI);
    model(1, 1) = (float)cos(rotation_angle / 180.0 * MY_PI);

    return model;
}

/**
 * @brief 扩展功能呢，绕任意轴旋转
 * 作用，绕任何过原点的轴进行旋转
 * @param axis
 * @param rotation_angle
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f get_model_matrix(Vector3f axis, float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // R(n,a) = I + sin(a)N + (1-cos(a))NN^T
    // N = (x,y,z)
    // N^T = (x,y,z)^T
    // N*N^T = (x^2,x*y,x*z;x*y,y^2,y*z;x*z,y*z,z^2)
    float theta = float(rotation_angle / 180.0 * MY_PI);
    float nx = axis[0], ny = axis[1], nz = axis[2];

    float cosa = cos(theta);
    float sina = sin(theta);

    model << nx * nx * (1 - cosa) + cosa, nx * ny * (1 - cosa) - nz * sina, nx * nz * (1 - cosa) + ny * sina, 0,
        nx * ny * (1 - cosa) + nz * sina, ny * ny * (1 - cosa) + cosa, ny * nz * (1 - cosa) - nx * sina, 0,
        nx * nz * (1 - cosa) - ny * sina, ny * nz * (1 - cosa) + nx * sina, nz * nz * (1 - cosa) + cosa, 0,
        0, 0, 0, 1;

    return model;
}

/**
 * @brief Get the projection matrix object
 * 使用给定的参数逐个元素地构建透视投影矩阵并返回该矩阵。
 * @param eye_fov
 * @param aspect_ratio
 * @param zNear
 * @param zFar
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // 透视投影矩阵
    // | 1/(aspect_ratio*t)     0               0                       0 |
    // | 0                      tan(fov/2)      0                       0 |
    // | 0                      0               (n+f)/(n-f)     2nf/(n-f) |
    // | 0                      0               -1                      0 |
    float fov = eye_fov / 180.0 * MY_PI;
    float t = float(tan(fov / 2.0));
    float n = zNear;
    float f = zFar;

    projection(0, 0) = float(1.0 / (aspect_ratio * t));
    projection(1, 1) = float(1.0 / t);
    projection(2, 2) = (n + f) / (n - f);
    projection(2, 3) = float(2.0 * n * f / (n - f));
    projection(3, 2) = -1.0;
    projection(3, 3) = 0.0;

    return projection;
}

/**
 * @brief Get the rotation object
 * 得到绕任意过原点的轴的旋转变换矩阵。
 * @param axis
 * @param angle
 * @return Eigen::Matrix4f
 */

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();

    return rotation;
}

int main(int argc, const char **argv)
{
    bool isExtend = false;

    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (isExtend)
    {
        if (argc >= 3)
        {
            command_line = true;
            angle = std::stof(argv[2]); // -r by default
            if (argc == 4)
            {
                filename = std::string(argv[3]);
            }
            else
                return 0;
        }

        rst::rasterizer r(700, 700);

        Eigen::Vector3f eye_pos = {0, 0, 5};

        std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

        std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

        auto pos_id = r.load_positions(pos);
        auto ind_id = r.load_indices(ind);

        int key = 0;
        int frame_count = 0;

        if (command_line)
        {
            r.clear(rst::Buffers::Color | rst::Buffers::Depth);

            r.set_model(get_model_matrix(angle));
            r.set_view(get_view_matrix(eye_pos));
            r.set_projection(get_projection_matrix(45, 1, 0.1f, 50));

            r.draw(pos_id, ind_id, rst::Primitive::Triangle);
            cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
            image.convertTo(image, CV_8UC3, 1.0f);

            cv::imwrite(filename, image);

            return 0;
        }

        while (key != 27)
        {
            r.clear(rst::Buffers::Color | rst::Buffers::Depth);

            r.set_model(get_model_matrix(angle));
            r.set_view(get_view_matrix(eye_pos));
            r.set_projection(get_projection_matrix(45, 1, 0.1f, 50));

            r.draw(pos_id, ind_id, rst::Primitive::Triangle);

            cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
            image.convertTo(image, CV_8UC3, 1.0f);
            cv::imshow("image", image);
            key = cv::waitKey(10);

            std::stringstream stream;
            float angle_mod = fmod(angle, 360.0f);
            stream << std::fixed << std::setprecision(2) << angle_mod;
            cv::setWindowTitle("image", "image\t" + stream.str());

            std::cout << "key: " << key << '\n';
            std::cout << "frame count: " << frame_count++ << '\n';

            switch (key)
            {
            case 'a':
                angle += 5;
                break;
            case 'd':
                angle -= 5;
                break;
            default:
                break;
            }
        }
    }

    else
    {
        // axis 'xXyYzZ' -> move axis
        Vector3f axis = {0, 1, 0};

        rst::rasterizer r(700, 700);

        Eigen::Vector3f eye_pos = {0, 0, 5};

        std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

        std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

        auto pos_id = r.load_positions(pos);
        auto ind_id = r.load_indices(ind);

        int key = 0;
        int frame_count = 0;

        while (key != 27)
        {
            r.clear(rst::Buffers::Color | rst::Buffers::Depth);

            r.set_model(get_model_matrix(axis, angle));
            r.set_view(get_view_matrix(eye_pos));
            r.set_projection(get_projection_matrix(45, 1, 0.1f, 50));

            r.draw(pos_id, ind_id, rst::Primitive::Triangle);

            cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
            image.convertTo(image, CV_8UC3, 1.0f);
            cv::imshow("image", image);
            key = cv::waitKey(10);

            std::string title = "";

            float angle_mod = fmod(angle, 360.0f);
            title += ("angle:" + converter_to_str(angle_mod, 2));
            if (angle_mod == 0)
                angle = 0;

            title += (" axis: (" + converter_to_str(axis.x(), 2) + "," + converter_to_str(axis.y(), 2) + "," + converter_to_str(axis.z(), 2) + " )");

            cv::setWindowTitle("image", "" + title);

            switch (key)
            {
            case 'a':
                angle += 5;
                break;
            case 'd':
                angle -= 5;
                break;
            case 'x':
                axis.x() += 0.1f;
                break;
            case 'X':
                axis.x() -= 0.1f;
                break;
            case 'y':
                axis.y() += 0.1f;
                break;
            case 'Y':
                axis.y() -= 0.1f;
                break;
            case 'z':
                axis.z() += 0.1f;
                break;
            case 'Z':
                axis.z() -= 0.1f;
                break;
            default:
                break;
            }

            // std::cout << "key: " << key << '\n';
            // std::cout << "frame count: " << frame_count++ << '\n';

            std::cout << "axis: " << axis << '\n';
        }

        return 0;
    }
}
