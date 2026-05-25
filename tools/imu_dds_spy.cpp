// Suscriptor DDS puro — sin ROS 2, sin rclcpp
// Demuestra que /imu/data viaja como mensaje DDS estándar
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>

#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <atomic>
#include <csignal>
#include <cstring>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

std::atomic<bool> running{true};
void signal_handler(int) { running = false; }

struct Vector3   { double x, y, z; };
struct Quaternion { double x, y, z, w; };

// Buffer crudo que recibe bytes CDR del sample
struct RawSample {
    std::vector<uint8_t> buf;
};

// TypeSupport mínimo: solo copia los bytes CDR tal cual
class RawType : public TopicDataType {
public:
    RawType() {
        set_name("sensor_msgs::msg::dds_::Imu_");
        max_serialized_type_size = 4096;
        is_compute_key_provided = false;
    }

    bool serialize(const void* const, SerializedPayload_t&, DataRepresentationId_t) override {
        return true;
    }

    bool deserialize(SerializedPayload_t& payload, void* data) override {
        auto* s = static_cast<RawSample*>(data);
        s->buf.resize(payload.length);
        memcpy(s->buf.data(), payload.data, payload.length);
        return true;
    }

    uint32_t calculate_serialized_size(const void* const, DataRepresentationId_t) override {
        return max_serialized_type_size;
    }

    void* create_data() override  { return new RawSample(); }
    void  delete_data(void* data) override { delete static_cast<RawSample*>(data); }

    bool compute_key(SerializedPayload_t&, InstanceHandle_t&, bool) override { return false; }
    bool compute_key(const void* const, InstanceHandle_t&, bool) override    { return false; }
};

class ImuListener : public DataReaderListener {
public:
    void on_data_available(DataReader* reader) override {
        RawSample sample;
        SampleInfo  info;

        while (reader->take_next_sample(&sample, &info) == RETCODE_OK) {
            if (!info.valid_data || sample.buf.size() < 80) continue;

            const uint8_t* b = sample.buf.data();
            size_t off = 4; // saltar cabecera CDR (4 bytes)

            // header.stamp
            uint32_t sec, nsec;
            memcpy(&sec,  b + off, 4); off += 4;
            memcpy(&nsec, b + off, 4); off += 4;

            // header.frame_id (string: 4-byte len + chars + padding CDR)
            uint32_t flen;
            memcpy(&flen, b + off, 4); off += 4 + flen;
            if (off % 4) off += 4 - (off % 4);

            // orientation (x,y,z,w) — 4×8 bytes
            Quaternion q;
            memcpy(&q.x, b+off, 8); off+=8;
            memcpy(&q.y, b+off, 8); off+=8;
            memcpy(&q.z, b+off, 8); off+=8;
            memcpy(&q.w, b+off, 8); off+=8;

            off += 9*8; // orientation_covariance

            // angular_velocity (x,y,z) — 3×8 bytes
            Vector3 gyro;
            memcpy(&gyro.x, b+off, 8); off+=8;
            memcpy(&gyro.y, b+off, 8); off+=8;
            memcpy(&gyro.z, b+off, 8); off+=8;

            off += 9*8; // angular_velocity_covariance

            // linear_acceleration (x,y,z)
            Vector3 accel;
            memcpy(&accel.x, b+off, 8); off+=8;
            memcpy(&accel.y, b+off, 8); off+=8;
            memcpy(&accel.z, b+off, 8); off+=8;

            std::cout << std::fixed << std::setprecision(4)
                      << "[DDS] t=" << sec << "." << std::setw(9) << std::setfill('0') << nsec
                      << "  accel=("  << accel.x << ", " << accel.y << ", " << accel.z << ") m/s²"
                      << "  gyro=("   << gyro.x  << ", " << gyro.y  << ", " << gyro.z  << ") rad/s"
                      << "  quat=(" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << ")\n";
        }
    }
};

int main() {
    signal(SIGINT, signal_handler);

    DomainParticipantQos pqos;
    pqos.name("imu_dds_spy_standalone");
    auto* participant = DomainParticipantFactory::get_instance()
                            ->create_participant(0, pqos);
    if (!participant) { std::cerr << "Error creando participante\n"; return 1; }

    // Registrar tipo raw
    TypeSupport ts(new RawType());
    ts.register_type(participant);

    auto* subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);

    auto* topic = participant->create_topic(
        "rt/imu/data",
        "sensor_msgs::msg::dds_::Imu_",
        TOPIC_QOS_DEFAULT);
    if (!topic) { std::cerr << "Error creando topic\n"; return 1; }

    DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
    rqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
    rqos.durability().kind  = VOLATILE_DURABILITY_QOS;
    rqos.history().kind     = KEEP_LAST_HISTORY_QOS;
    rqos.history().depth    = 10;

    ImuListener listener;
    auto* reader = subscriber->create_datareader(topic, rqos, &listener);
    if (!reader) { std::cerr << "Error creando DataReader\n"; return 1; }

    std::cout << "=== IMU DDS Spy (standalone — sin ROS 2) ===\n";
    std::cout << "Suscrito a: rt/imu/data [sensor_msgs::msg::dds_::Imu_]\n";
    std::cout << "Dominio DDS: 0 | Ctrl+C para salir\n\n";

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    DomainParticipantFactory::get_instance()->delete_participant(participant);
    return 0;
}
