#pragma once

// ---------------------------------------------------------------------------
// Types that state their own stored fields
// ---------------------------------------------------------------------------
//
// A type that gets written to a settings file names its stored fields inside itself:
//
//   struct options_t
//   {
//       double min_area_px{ 60.0 };
//
//       DECLARE_SERIALIZABLE_FIELDS(
//           v("min_area_px", o.min_area_px);
//       )
//   };
//
// The list sits inside the type so that whoever adds a field to the struct has it in view, which
// is the guard against a field that quietly stops being kept.
//
// It names keys and members and nothing else, so it states no file format. Whoever stores the type
// supplies the visitor and owns the format; for this project that is the JSON engine in
// `app_config.cc`.
//
// A list may name a `static constexpr` member, which reaches the visitor as a const field. A
// reader cannot write into one, so it is a stamp: the file states it and has to agree, which is
// how a schema version travels with the settings it describes.

// Declares that list. `v` is the visitor and `o` the object being visited, so a body calls
// `v("<key>", o.<member>);` once per stored field. `Self` deduces to `T&` and `const T&` alike,
// which is what lets the one list serve reading and writing.
#define DECLARE_SERIALIZABLE_FIELDS(...)                 \
    template <typename V, typename Self>                 \
    static void visit_serializable_fields(V& v, Self& o) \
    {                                                    \
        __VA_ARGS__                                      \
    }

namespace utils
{
    namespace detail
    {
        // Accepts any (key, field) pair, which is what tells a field list apart from anything else
        // that happens to take two arguments.
        struct serializable_field_probe_t { void operator()(const char*, auto&) const {} };
    }

    // A type whose stored fields are declared with DECLARE_SERIALIZABLE_FIELDS.
    template <typename T>
    concept has_serializable_fields = requires(detail::serializable_field_probe_t& probe, T& object) {
        T::visit_serializable_fields(probe, object);
    };

} // namespace utils
