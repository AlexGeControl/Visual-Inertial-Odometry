#include "utilities.h"

#include "CSVWriter.h"
#include "CSVParser.h"

void SavePoints(
    const std::string &filename, 
    const std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> &points
) {
    // init writer:
    CSVWriter csv(",");
    csv.enableAutoNewRow(4);

    // write header:
    csv << "x" 
        << "y" 
        << "z" 
        << "lambda";

    // write points:
    for (const Eigen::Vector4d &p: points) {
        csv << p(0)
            << p(1)
            << p(2)
            << p(3);
    }

    // save to persistent storage:
    csv.writeToFile(filename);
}

void SaveFeatures(
    const std::string &filename,
    const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &points,
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &features
) {
    // init writer:
    CSVWriter csv(",");
    csv.enableAutoNewRow(6);

    // write header:
    csv << "x" 
        << "y" 
        << "z" 
        << "lambda"
        << "x_normalized"
        << "y_normalized";
    
    // write landmark position and corresponding observation in normalized plane:
    const size_t N = points.size();
    for (size_t i = 0; i < N; ++i) {
        const Eigen::Vector4d &p = points.at(i);
        const Eigen::Vector2d &f = features.at(i);

        csv << p(0)
            << p(1)
            << p(2)
            << p(3)
            << f(0)
            << f(1);
    }

    // save to persistent storage:
    csv.writeToFile(filename);
}

void SaveLines(
    const std::string &filename,
    const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &features
) {
    // init writer:
    CSVWriter csv(",");
    csv.enableAutoNewRow(4);

    // write header:
    csv << "x1_normalized" 
        << "y1_normalized"
        << "x2_normalized"
        << "y2_normalized";

    // write line endpoint observations in normalized plane:
    for (const Eigen::Vector4d &f: features) {
        csv << f(0)
            << f(1)
            << f(2)
            << f(3);
    }

    // save to persistent storage:
    csv.writeToFile(filename);
}

void LoadPose(
    const std::string &filename, 
    std::vector<MotionData>& poses
) {
    io::CSVReader<14> in(filename);

    // config columns to be parsed:
    in.read_header(
        io::ignore_extra_column, 
        "timestamp",
        "q_w", "q_x", "q_y", "q_z",
        "p_x", "p_y", "p_z",
        "gyro_x", "gyro_y", "gyro_z",
        "acc_x", "acc_y", "acc_z"
    );

    double timestamp;
    double q_w, q_x, q_y, q_z;
    double p_x, p_y, p_z;
    double gyro_x, gyro_y, gyro_z;
    double acc_x, acc_y, acc_z;

    while(
        in.read_row(
            timestamp, 
            q_w, q_x, q_y, q_z,
            p_x, p_y, p_z,
            gyro_x, gyro_y, gyro_z,
            acc_x, acc_y, acc_z
        )
    ){
        MotionData data;

        Eigen::Quaterniond q(q_w, q_x, q_y, q_z);
        Eigen::Vector3d t(p_x, p_y, p_z);
        Eigen::Vector3d gyro(gyro_x, gyro_y, gyro_z);
        Eigen::Vector3d acc(acc_x, acc_y, acc_z);

        data.timestamp = timestamp;

        data.Rwb = Eigen::Matrix3d(q);
        data.twb = t;
        data.imu_gyro = gyro;
        data.imu_acc = acc;

        poses.push_back(data);
    }
}

void SavePose(
    const std::string &filename, 
    const std::vector<MotionData> &poses
) {
    // init writer:
    CSVWriter csv(",");
    csv.enableAutoNewRow(14);

    // write header:
    csv << "timestamp" 
        // orientation as quaternion:
        << "q_w"
        << "q_x"
        << "q_y"
        << "q_z"
        // position:
        << "p_x"
        << "p_y"
        << "p_z"
        // gyroscope measurements:
        << "gyro_x"
        << "gyro_y"
        << "gyro_z"
        // accelerometer measurements:
        << "acc_x"
        << "acc_y"
        << "acc_z";

    // write poses:
    for (size_t i = 0; i < poses.size(); ++i) {
        const MotionData &data = poses.at(i);

        double time = data.timestamp;
        
        Eigen::Quaterniond q(data.Rwb);
        const Eigen::Vector3d &t = data.twb;
        const Eigen::Vector3d &gyro = data.imu_gyro;
        const Eigen::Vector3d &acc = data.imu_acc;

        csv << time
            // orientation as quaternion:
            << q.w()
            << q.x()
            << q.y()
            << q.z()
            // position:
            << t.x()
            << t.y()
            << t.z()
            // gyroscope measurements:
            << gyro.x()
            << gyro.y()
            << gyro.z()
            // accelerometer measurements:
            << acc.x()
            << acc.y()
            << acc.z();
    }

    // save to persistent storage:
    csv.writeToFile(filename);
}

void SavePoseTUM(
    const std::string &filename, 
    const std::vector<MotionData> &poses
) {
    // init writer:
    CSVWriter csv(",");
    csv.enableAutoNewRow(8);

    // write header:
    csv << "timestamp" 
        // position:
        << "p_x"
        << "p_y"
        << "p_z"
        // orientation as quaternion:
        << "q_x"
        << "q_y"
        << "q_z"
        << "q_w";

    // write poses:
    for (size_t i = 0; i < poses.size(); ++i) {
        const MotionData &data = poses.at(i);

        double time = data.timestamp;

        const Eigen::Vector3d &t = data.twb;
        Eigen::Quaterniond q(data.Rwb);

        csv << time
            // position:
            << t.x()
            << t.y()
            << t.z()
            // orientation as quaternion:
            << q.x()
            << q.y()
            << q.z()
            << q.w();
    }

    // save to persistent storage:
    csv.writeToFile(filename);
}