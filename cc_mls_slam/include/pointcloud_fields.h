#pragma once
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

// Checked scalar reader: supports organized clouds, padding and unaligned fields.
namespace cc_mls {
class CloudFields {
    const sensor_msgs::PointCloud2& cloud_;
public:
    explicit CloudFields(const sensor_msgs::PointCloud2& cloud):cloud_(cloud) {
        const uint64_t row_bytes=uint64_t(cloud.width)*cloud.point_step;
        const uint64_t total_bytes=uint64_t(cloud.height)*cloud.row_step;
        if (!cloud.point_step || row_bytes>cloud.row_step || total_bytes>cloud.data.size())
            throw std::invalid_argument("Invalid PointCloud2 storage/stride");
    }
    const sensor_msgs::PointField& field(const std::string& name) const {
        for(const auto& f:cloud_.fields) if(f.name==name) {
            const unsigned size=scalarSize(f.datatype);
            if(f.count!=1 || uint64_t(f.offset)+size>cloud_.point_step)
                throw std::invalid_argument("Invalid field layout: "+name);
            return f;
        }
        throw std::invalid_argument("Missing PointCloud2 field: "+name);
    }
    static unsigned scalarSize(uint8_t type) {
        using F=sensor_msgs::PointField;
        switch(type) {
            case F::INT8: case F::UINT8: return 1;
            case F::INT16: case F::UINT16: return 2;
            case F::INT32: case F::UINT32: case F::FLOAT32: return 4;
            case F::FLOAT64: return 8;
            default: throw std::invalid_argument("Unsupported PointCloud2 datatype");
        }
    }
    template<class T> T scalar(const uint8_t* data) const {
        uint8_t bytes[sizeof(T)]; std::memcpy(bytes,data,sizeof(T));
        const uint16_t one=1;
        const bool host_big=*reinterpret_cast<const uint8_t*>(&one)==0;
        if(host_big!=cloud_.is_bigendian) std::reverse(bytes,bytes+sizeof(T));
        T value; std::memcpy(&value,bytes,sizeof(T)); return value;
    }
    double value(size_t index,const sensor_msgs::PointField& f) const {
        if(!cloud_.width || index>=uint64_t(cloud_.width)*cloud_.height)
            throw std::out_of_range("Point index outside cloud");
        const auto* data=cloud_.data.data()+(index/cloud_.width)*cloud_.row_step+
                         (index%cloud_.width)*cloud_.point_step+f.offset;
        using F=sensor_msgs::PointField;
        switch(f.datatype) {
            case F::INT8: return scalar<int8_t>(data);
            case F::UINT8: return scalar<uint8_t>(data);
            case F::INT16: return scalar<int16_t>(data);
            case F::UINT16: return scalar<uint16_t>(data);
            case F::INT32: return scalar<int32_t>(data);
            case F::UINT32: return scalar<uint32_t>(data);
            case F::FLOAT32: return scalar<float>(data);
            case F::FLOAT64: return scalar<double>(data);
            default: throw std::invalid_argument("Unsupported field datatype");
        }
    }
};
}
