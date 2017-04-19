#include <util/tiny_logger.hpp>

#include <3dvideo/format.hpp>
#include <3dvideo/dataset_input.hpp>


DatasetInput::DatasetInput(const std::string &path)
    : in(path, std::ios::binary)
{
}

Status DatasetInput::readHeader()
{
    if (!in.is_open())
        return Status::ERROR;

    Field magic;
    binRead(magic);
    if (magic != Field::MAGIC)
    {
        TLOG(ERROR) << "Invalid header, could not read dataset!";
        return Status::ERROR;
    }

    return Status::SUCCESS;
}

Status DatasetInput::readFrame()
{
    return Status::SUCCESS;
}
