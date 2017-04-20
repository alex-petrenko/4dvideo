#include <util/tiny_logger.hpp>

#include <3dvideo/dataset_input.hpp>


DatasetInput::DatasetInput(const std::string &path)
    : in(path, std::ios::binary)
{
    // populating metadata parsers
    auto &mp = metadataParsers;
    mp[Field::VERSION] = [&] { return binRead(meta.formatVersion); };

    mp[Field::COLOR_RESOLUTION] = [&] { return binRead(meta.color.w, meta.color.h); };
    mp[Field::DEPTH_RESOLUTION] = [&] { return binRead(meta.depth.w, meta.depth.h); };

    mp[Field::COLOR_INTRINSICS] = [&] { return binRead(meta.color.f, meta.color.cx, meta.color.cy); };
    mp[Field::DEPTH_INTRINSICS] = [&] { return binRead(meta.depth.f, meta.depth.cx, meta.depth.cy); };

    mp[Field::COLOR_FORMAT] = [&] { return binRead(meta.colorFormat); };
    mp[Field::DEPTH_FORMAT] = [&] { return binRead(meta.depthFormat); };

    mp[Field::DATA_SECTION] = [&] { return true; };
}

Status DatasetInput::readHeader()
{
    if (!in.is_open())
        return Status::ERROR;

    Field field;
    bool ok = binRead(field);
    if (!ok || field != Field::MAGIC)
    {
        TLOG(ERROR) << "Invalid header, could not start reading header!";
        return Status::ERROR;
    }

    ok = binRead(field);
    if (!ok || field != Field::METADATA_SECTION)
    {
        TLOG(ERROR) << "Could not find metadata section";
        return Status::ERROR;
    }

    while (ok && field != Field::DATA_SECTION)
        ok = readMetadataField(field);

    if (!ok)
    {
        TLOG(ERROR) << "Could not read field " << int(field);
        return Status::ERROR;
    }

    TLOG(INFO) << "Format version is: " << meta.formatVersion;

    return Status::SUCCESS;
}

bool DatasetInput::readMetadataField(Field &field)
{
    const auto ok = binRead(field);
    if (!ok)
    {
        TLOG(ERROR) << "Could not find the next metadata field";
        return false;
    }

    const auto it = metadataParsers.find(field);
    if (it == metadataParsers.end())
    {
        TLOG(ERROR) << "Could not find parser for field " << int(field);
        return false;
    }

    const auto &parser = it->second;
    return parser();
}

Status DatasetInput::readFrame()
{
    return Status::SUCCESS;
}


