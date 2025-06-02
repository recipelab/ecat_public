#include <boost/preprocessor/seq/for_each.hpp>
#include <string>

#define REALLY_MAKE_STRING(x) #x
#define MAKE_STRING(x) REALLY_MAKE_STRING(x)
#define MACRO1(r, data, elem) elem,
#define MACRO1_STRING(r, data, elem)    case elem: return REALLY_MAKE_STRING(elem);
#define MACRO1_ENUM(r, data, elem)      if (REALLY_MAKE_STRING(elem) == eStrEl) return elem;

#define MakeEnum(eName, SEQ) \
    enum eName { BOOST_PP_SEQ_FOR_EACH(MACRO1, , SEQ) \
    last_##eName##_enum}; \
    const int eName##Count = BOOST_PP_SEQ_SIZE(SEQ); \
    static std::string eName##2String(const enum eName eel) \
    { \
        switch (eel) \
        { \
        BOOST_PP_SEQ_FOR_EACH(MACRO1_STRING, , SEQ) \
        default: return "Unknown enumerator value."; \
        }; \
    }; \
    static enum eName eName##2Enum(const std::string eStrEl) \
    { \
        BOOST_PP_SEQ_FOR_EACH(MACRO1_ENUM, , SEQ) \
        return (enum eName)0; \
    };

#define EnumName(Tuple)                 BOOST_PP_TUPLE_ELEM(2, 0, Tuple)
#define EnumValue(Tuple)                BOOST_PP_TUPLE_ELEM(2, 1, Tuple)
#define MACRO2(r, data, elem)           EnumName(elem) EnumValue(elem),
#define MACRO2_STRING(r, data, elem)    case EnumName(elem): return BOOST_PP_STRINGIZE(EnumName(elem));

#define MakeEnumEx(eName, SEQ) \
    enum eName { BOOST_PP_SEQ_FOR_EACH(MACRO2, _, SEQ) \
    last_##eName##_enum }; \
    const int eName##Count = BOOST_PP_SEQ_SIZE(SEQ); \
    static std::string eName##2String(const enum eName eel) \
    { \
        switch (eel) \
        { \
        BOOST_PP_SEQ_FOR_EACH(MACRO2_STRING, _, SEQ) \
        default: return "Unknown enumerator value."; \
        }; \
    };