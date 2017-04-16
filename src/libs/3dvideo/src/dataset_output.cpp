#include <3dvideo/format.hpp>
#include <3dvideo/dataset_output.hpp>


DatasetOutput::DatasetOutput(const std::string &path)
    : out(path, std::ios::binary)
{
}

Status DatasetOutput::writeHeader()
{
    if (!out.is_open())
        return Status::ERROR;

    binWrite(Field::MAGIC);

    writeField(Field::VERSION, Format::CURRENT_VERSION);

    binWrite(Field::METADATA_SECTION);

    // here goes dataset metadata

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
