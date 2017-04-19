#include <3dvideo/dataset_reader.hpp>


DatasetReader::DatasetReader(const std::string &path, const CancellationToken &cancellationToken)
    : FrameProducer(cancellationToken)
    , dataset(path)
{
}

DatasetReader::~DatasetReader()
{
}

void DatasetReader::init()
{
    dataset.readHeader();
}

void DatasetReader::run()
{
}
