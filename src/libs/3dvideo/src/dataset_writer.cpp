#include <3dvideo/dataset_writer.hpp>


DatasetWriter::DatasetWriter(FrameQueue &q, CancellationToken &cancel)
    : Consumer(q, cancel)
    , dataset(R"(C:\temp\tst\dataset.4dv)")
{
    dataset.writeHeader();
}

DatasetWriter::~DatasetWriter()
{
}

void DatasetWriter::process(std::shared_ptr<Frame> &frame)
{
    dataset.writeFrame(*frame);
}
