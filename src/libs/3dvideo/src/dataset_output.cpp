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

    const uint32_t metadataSize = uint32_t(Format::METADATA_SIZE);
    writeField(Field::METADATA_SIZE, metadataSize);
    // METADATA_SIZE bytes reserved for metadata
    std::vector<uchar> metadataBuff(metadataSize, 0);
    writeField(Field::METADATA_SECTION, (char *)metadataBuff.data(), metadataBuff.size());

    return Status::SUCCESS;
}

Status DatasetOutput::writeFrame(const Frame &frame)
{
    binWrite(Field::FRAME_SECTION);
    writeField(Field::FRAME_NUMBER, frame.frameNumber);

    return Status::SUCCESS;
}
