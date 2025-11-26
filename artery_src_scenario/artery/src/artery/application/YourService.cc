#include "YourService.h"
// #include "artery/application/YourServiceMessage_m.h"              // make sure this is here
#include "artery/traci/VehicleController.h"
#include <omnetpp/cpacket.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>

using namespace omnetpp;
using namespace vanetza;

namespace artery
{

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");

Define_Module(YourService)

// --- ctor / dtor ---------------------------------------------------------

YourService::YourService() :
    m_self_msg(nullptr)                     // important: initialize to nullptr
{
}

YourService::~YourService()
{
    EV_INFO << "Closing socket..." << endl;
    this->socket.close();
    EV_INFO << "Closing context..." << endl;
    this->context.close();
    if (m_self_msg != nullptr) {           // guard against nullptr
        cancelAndDelete(m_self_msg);
        m_self_msg = nullptr;
    }
}

// --- BTP indication ------------------------------------------------------

void YourService::indicate(const btp::DataIndication& ind, cPacket* packet, const NetworkInterface& net)
{
    Enter_Method("indicate");

    auto msg = dynamic_cast<YourServiceMessage*>(packet);
    if (msg != nullptr) {
        EV_INFO << "Received YourServiceMessage on channel " << net.channel << "\n";
        EV_INFO << "  messageType = "       << msg->getMessageType() << "\n";
        EV_INFO << "  vehicleId = "         << msg->getVehicleId() << "\n";
        EV_INFO << "  sensorId = "          << msg->getSensorId() << "\n";
        EV_INFO << "  initialTimestamp = "  << msg->getInitialTimestamp() << "\n";
        EV_INFO << "  distance = "          << msg->getDistance() << "\n";
        EV_INFO << "  x = "                 << msg->getX() << "\n";
        EV_INFO << "  y = "                 << msg->getY() << "\n";
    } else {
        EV_WARN << "Received unknown packet type (byteLength=" << packet->getByteLength() << ")\n";
    }

    delete packet;
}

// --- initialize / finish -------------------------------------------------

void YourService::initialize()
{
    ItsG5Service::initialize();

    // create self-message and schedule first trigger
    m_self_msg = new cMessage("YourServiceTimer");
    subscribe(scSignalCamReceived);

    scheduleAt(simTime() + 3.0, m_self_msg);     // now m_self_msg is valid

    // Connect to ZMQ
    this->context = zmq::context_t{1};
    this->socket = zmq::socket_t{context, ZMQ_SUB};
    this->socket.setsockopt(ZMQ_RCVTIMEO, 100);
    this->socket.setsockopt(ZMQ_SNDTIMEO, 100);

    zmq_setsockopt(this->socket, ZMQ_SUBSCRIBE, topic.c_str(), topic.length());

    std::string host = par("host").stringValue();
    int port = par("port").intValue();
    std::string addr = "tcp://" + host + ":" + std::to_string(port);
    this->socket.connect(addr);
}

void YourService::finish()
{
    ItsG5Service::finish();
}

// --- handleMessage / timer ----------------------------------------------

void YourService::handleMessage(cMessage* msg)
{
    Enter_Method("handleMessage");

    if (msg == m_self_msg) {
        EV_INFO << "YourService self message at t=" << simTime() << "\n";

        // do periodic work
        trigger();

        // reschedule same self-message
        scheduleAt(simTime() + 3.0, m_self_msg);
    } else {
        // let base class handle other messages (BTP, timers, etc.)
        ItsG5Service::handleMessage(msg);
    }
}

// --- main logic (ZMQ + send) --------------------------------------------

void YourService::trigger()
{
    Enter_Method("trigger");

    static const vanetza::ItsAid example_its_aid = 16480;

    auto& mco = getFacilities().get_const<MultiChannelPolicy>();
    auto& networks = getFacilities().get_const<NetworkInterfaceTable>();
    auto& vehicleController = getFacilities().get_const<traci::VehicleController>();

    for (auto channel : mco.allChannels(example_its_aid)) {
        auto network = networks.select(channel);
        if (network) {
            btp::DataRequestB req;
            req.destination_port = host_cast(getPortNumber(channel));
            req.gn.transport_type = geonet::TransportType::SHB;
            req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
            req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
            req.gn.its_aid = example_its_aid;

            this->socket.setsockopt(ZMQ_RCVTIMEO, 400);

            zmq::message_t reply{};
            int sndhwm;
            size_t sndhwm_size = sizeof(sndhwm);
            int rc;
            std::string uk = "null";
            std::list<int> sensors_with_input;

            do {
                socket.recv(reply, zmq::recv_flags::dontwait);
                rc = zmq_getsockopt(socket, ZMQ_RCVMORE, &sndhwm, &sndhwm_size);
                if (reply.to_string().empty())
                    break;
                if (reply.to_string()[0] == '{'){
                    json jsonResp = json::parse(reply.to_string());
                    std::string message_type = jsonResp["message_type"].get<std::string>();
                    EV << "message_type:" << message_type << endl;
                    double vehicle_id = jsonResp["vehicle_id"].get<double>();
                    EV << "vehicle_id:" << vehicle_id << endl;
                    double sensor_id = jsonResp["sensor_id"].get<double>();
                    EV << "sensor_id:"<< sensor_id << endl;
                    double timestamp = jsonResp["timestamp"].get<double>();
                    EV << "timestamp:" << timestamp << endl;
                    double distance = jsonResp["distance"].get<double>();
                    EV << "distance:" << distance << endl;

                    EV << "    OBST-" << sensor_id << "<" << "@" << vehicle_id
                    << " (t=" << timestamp << ")  Distance=" << distance << "m\n";

                    double vehicle_length  = vehicleController.getLength().value();
                    double vehicle_width   = vehicleController.getWidth().value(); // unused but ok
                    double vehicle_angle   = vehicleController.getHeading().radian();

                    omnetpp::cFigure::Point cur_pos_center(
                        vehicleController.getPosition().x.value(),
                        vehicleController.getPosition().y.value());

                    double depth = distance;

                    // detection in vehicle-local coordinates (relative to center)
                    double det_x_v = vehicle_length / 2.0 + depth;
                    double det_y_v = 0.0;

                    double cosH = std::cos(vehicle_angle);
                    double sinH = std::sin(vehicle_angle);

                    double det_dx =  cosH * det_x_v - sinH * det_y_v;
                    double det_dy =  sinH * det_x_v + cosH * det_y_v;

                    omnetpp::cFigure::Point detection_coord =
                        cur_pos_center + omnetpp::cFigure::Point(det_dx, det_dy);

                    // sensor at front center
                    omnetpp::cFigure::Point sensor_pos =
                        cur_pos_center + omnetpp::cFigure::Point(vehicle_length / 2.0, 0.0);

                    omnetpp::cFigure::Point detection_pos(depth, 0.0);
                    omnetpp::cFigure::Point coord = sensor_pos + detection_pos;

                    auto* packet = new YourServiceMessage("YourService Packet");
                    packet->setMessageType(message_type.c_str());
                    packet->setVehicleId(vehicle_id);
                    packet->setSensorId(sensor_id);
                    packet->setInitialTimestamp(timestamp);
                    packet->setDistance(distance);
                    packet->setX(coord.x);
                    packet->setY(coord.y);
                    packet->setByteLength(42);

                    EV << "Sent via YourService:"
                    << " Type: "     << message_type
                    << ", VehicleID: " << vehicle_id
                    << ", SensorID: "  << sensor_id
                    << ", Timestamp: " << timestamp
                    << ", Distance: "  << distance
                    << ", CoordX: "    << coord.x
                    << ", CoordY: "    << coord.y << endl;

                    request(req, packet, network.get());
                } else {
                    EV << "STRING=" << reply.to_string() << endl;
                    EV << "RCV_MORE=" << sndhwm << endl;
            }
                
            } while (true);
        } else {
            EV_ERROR << "No network interface available for channel " << channel << "\n";
        }
    }
}

// --- signal --------------------------------------------------------------

void YourService::receiveSignal(cComponent* source, simsignal_t signal, cObject*, cObject*)
{
    Enter_Method("receiveSignal");

    if (signal == scSignalCamReceived) {
        auto& vehicle = getFacilities().get_const<traci::VehicleController>();
        EV_INFO << "Vehicle " << vehicle.getVehicleId()
                << " received a CAM in sibling service\n";
    }
}

} // namespace artery
