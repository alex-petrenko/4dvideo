#include <3dvideo/format.hpp>
#include <3dvideo/dataset_output.hpp>


DatasetOutput::DatasetOutput(const std::string &path)
    : out(path, std::ios::binary)
{
}

Status DatasetOutput::writeHeader(const SensorManager &sensorManager)
{
    if (!out.is_open())
        return Status::ERROR;

    binWrite(Field::MAGIC);

    writeField(Field::VERSION, Format::CURRENT_VERSION);

    binWrite(Field::METADATA_SECTION);

    CameraParams color, depth;
    ColorDataFormat colorFormat;
    DepthDataFormat depthFormat;
    sensorManager.getColorParams(color, colorFormat);
    sensorManager.getDepthParams(depth, depthFormat);

    binWrite(Field::COLOR_RESOLUTION);
    binWrite(uint32_t(color.w)), binWrite(uint32_t(color.h));

    binWrite(Field::DEPTH_RESOLUTION);
    binWrite(uint32_t(depth.w)), binWrite(uint32_t(depth.h));

    binWrite(Field::COLOR_INTRINSICS);
    binWrite(color.f), binWrite(color.cx), binWrite(color.cy);

    binWrite(Field::DEPTH_INTRINSICS);
    binWrite(depth.f), binWrite(depth.cx), binWrite(depth.cy);

    writeField(Field::COLOR_FORMAT, colorFormat);
    writeField(Field::DEPTH_FORMAT, depthFormat);

    binWrite(Field::DATA_SECTION);

    return Status::SUCCESS;
}

Status DatasetOutput::writeFrame(const Frame &frame)
{
    binWrite(Field::FRAME_SECTION);
    writeField(Field::FRAME_NUMBER, frame.frameNumber);

    writeField(Field::COLOR, frame.color.data, frame.color.total());
    writeField(Field::DEPTH, frame.depth.data, frame.depth.total());

    return Status::SUCCESS;
}
