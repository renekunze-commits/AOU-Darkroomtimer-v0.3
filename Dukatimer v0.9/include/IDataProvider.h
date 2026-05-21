#pragma once

#include <cstddef>
#include <cstdint>

class IDataProvider
{
public:
    virtual ~IDataProvider() = default;

    virtual bool isDirty() const = 0;
    virtual void clearDirty() = 0;
    virtual void markDirty() = 0; // New method to restore dirty flag on failure

    virtual bool serialize(uint8_t **buffer, size_t *length, uint16_t *outHash) = 0;
    virtual bool deserialize(const uint8_t *buffer, size_t length, uint16_t hash) = 0;

    virtual const char *getFileName() const = 0;
};
