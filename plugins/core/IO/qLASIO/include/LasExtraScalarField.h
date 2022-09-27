#pragma once


#include <vector>
#include <string>

class QDataStream;

class ccPointCloud;
class ccScalarField;

struct laszip_header;
struct laszip_vlr;
typedef laszip_vlr laszip_vlr_struct;

/// This serves the same purpose as LasScalarField but for extra bytes
struct LasExtraScalarField
{
    /// Data types available LAS Extra Field
    enum DataType
    {
        Undocumented = 0,
        u8,
        i8,
        u16,
        i16,
        u32,
        i32,
        u64,
        i64,
        f32,
        f64,
        u8_2,
        i8_2,
        u16_2,
        i16_2,
        u32_2,
        i32_2,
        u64_2,
        i64_2,
        f32_2,
        f64_2,
        u8_3,
        i8_3,
        u16_3,
        i16_3,
        u32_3,
        i32_3,
        u64_3,
        i64_3,
        f32_3,
        f64_3,
        Invalid
    };

    enum Kind
    {
        Signed,
        Unsigned,
        Floating
    };

  public:
    explicit LasExtraScalarField(QDataStream &dataStream);
    void writeTo(QDataStream &dataStream) const;

  public: // Static Helper functions that works on collection of LasExtraScalarFields
    static std::vector<LasExtraScalarField> ParseExtraScalarFields(const laszip_header &laszipHeader);
    static std::vector<LasExtraScalarField> ParseExtraScalarFields(const laszip_vlr_struct &extraBytesVlr);
    static void InitExtraBytesVlr(laszip_vlr_struct &vlr,
                                  const std::vector<LasExtraScalarField> &extraFields);
    static void UpdateByteOffsets(std::vector<LasExtraScalarField> &extraFields);
    static unsigned int TotalExtraBytesSize(const std::vector<LasExtraScalarField> &extraScalarFields);
    static void MatchExtraBytesToScalarFields(std::vector<LasExtraScalarField> &extraScalarFields,
                                              const ccPointCloud &pointCloud);

  public: // methods
    // LAS Spec integer value for the type
    uint8_t typeCode() const;

    // Properties we can derive from the type attribute
    unsigned int elementSize() const;
    unsigned int numElements() const;
    unsigned int byteSize() const;
    Kind kind() const;
    const char *typeName() const;

    // Properties we can derive from the type options attribute
    bool noDataIsRelevant() const;
    bool minIsRelevant() const;
    bool maxIsRelevant() const;
    bool scaleIsRelevant() const;
    bool offsetIsRelevant() const;

    void resetScalarFieldsPointers();

    static DataType DataTypeFromValue(uint8_t value);

  public: // Data members
    // These fields are from the vlr itself
    DataType type{Undocumented};
    uint8_t options{0};
    char name[32] = "";
    char description[32] = "";
    uint8_t noData[3][8] = {0};
    uint8_t mins[3][8] = {0};
    uint8_t maxs[3][8] = {0};
    double scales[3] = {0.0};
    double offsets[3] = {0.0};

    // These are added by us
    unsigned int byteOffset{0};
    ccScalarField *scalarFields[3] = {nullptr};
    // TODO explain better
    // This strings store the name of the field in CC,
    // Extra fields name may clash with existing scalarfields name
    // (eg: the user calls one of his extra field "Intensity")
    // we have to alter the real name
    std::string ccName{};
};
