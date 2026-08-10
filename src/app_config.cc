#include "app_config.hh"

#include "utils/serializable.hh"

#include <nlohmann/json.hpp>

#include <concepts>
#include <format>
#include <fstream>
#include <limits>
#include <system_error>
#include <type_traits>

#ifdef _WIN32
#  include <windows.h> // GetModuleFileNameW
#endif

namespace app
{
    namespace
    {
        using json = nlohmann::ordered_json;

        // Deep enough to clear a build tree's out/build/<preset>/, stopping at the filesystem root.
        constexpr int kSearchLevels = 6;

        // The directory holding the running executable, empty where it cannot be determined.
        // Anchored to the executable so the same paths come out however the app is launched.
        std::filesystem::path exe_dir()
        {
#ifdef _WIN32
            std::wstring buf(MAX_PATH, L'\0');
            for (;;) {
                const DWORD written = ::GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
                if (written == 0) { break; }
                if (written < buf.size()) {
                    return std::filesystem::path{ buf.substr(0, written) }.parent_path();
                }
                buf.resize(buf.size() * 2); // a filled buffer means the path was truncated
            }
#endif
            return {};
        }

        // First existing `<dir>/relative` at or above the executable, else nullopt.
        std::optional<std::filesystem::path> search_above_exe(const std::filesystem::path& relative)
        {
            const std::filesystem::path start = exe_dir();
            if (start.empty()) { return std::nullopt; }

            std::error_code ec;
            std::filesystem::path dir = start;
            for (int level = 0; level < kSearchLevels && dir.has_relative_path(); ++level) {
                if (const std::filesystem::path candidate = dir / relative;
                    std::filesystem::exists(candidate, ec))
                {
                    return candidate;
                }
                dir = dir.parent_path();
            }
            return std::nullopt;
        }

        // =====================================================================================
        // Leaf conversions
        //
        // One pair per type a field list mentions but that carries no list of its own, because it
        // is a single value in the file. Nothing here knows a key name.
        //
        // The keys themselves are not spelled here: every stored type names its own beside its
        // members with `DECLARE_SERIALIZABLE_FIELDS` (see `utils/serializable.hh`), so a field and
        // its key are added in one place. What this file owns is the format those lists are
        // written in.
        // =====================================================================================

        template <typename T> struct is_duration : std::false_type {};
        template <typename R, typename P> struct is_duration<std::chrono::duration<R, P>> : std::true_type {};

        template <typename T> struct is_optional : std::false_type {};
        template <typename T> struct is_optional<std::optional<T>> : std::true_type {};

        json to_json(const auto& value);

        json to_json_leaf(const source_address& v)              { return json(v.to_string()); }
        json to_json_leaf(pose::view_plane_t v)                 { return json(std::string{ pose::view_plane_name(v) }); }
        json to_json_leaf(pose::tag_detector::pose_method_t v)  { return json(std::string{ pose::pose_method_to_str(v) }); }
        json to_json_leaf(pose::joint_side_t v)                 { return json(std::string{ pose::joint_side_name(v) }); }
        json to_json_leaf(marker_kind_t v)                      { return json(std::string{ marker_kind_name(v) }); }
        json to_json_leaf(const Eigen::Vector3d& v)             { return json::array({ v.x(), v.y(), v.z() }); }
        json to_json_leaf(const Eigen::Vector2d& v)             { return json::array({ v.x(), v.y() }); }
        json to_json_leaf(const Eigen::Vector2i& v)             { return json::array({ v.x(), v.y() }); }

        // Row major, so the file reads the way the matrix is written.
        json to_json_leaf(const Eigen::Matrix2d& v)
        {
            return json::array({ json::array({ v(0, 0), v(0, 1) }),
                                 json::array({ v(1, 0), v(1, 1) }) });
        }

        json to_json(const auto& value)
        {
            using T = std::remove_cvref_t<decltype(value)>;
            if constexpr (is_optional<T>::value) {
                return value.has_value() ? to_json(*value) : json(nullptr);
            }
            else if constexpr (is_duration<T>::value) {
                return json(value.count());
            }
            else if constexpr (utils::has_serializable_fields<T>) {
                json node = json::object();
                auto writer = [&node](const char* key, const auto& field) { node[key] = to_json(field); };
                T::visit_serializable_fields(writer, value);
                return node;
            }
            else if constexpr (requires { to_json_leaf(value); }) {
                return to_json_leaf(value);
            }
            else {
                return json(value); // arithmetic, bool, std::string
            }
        }

        // -------------------------------------------------------------------------------------

        std::string key_name(std::string_view group, const char* key)
        {
            return group.empty() ? std::string{ key } : std::format("{}.{}", group, key);
        }

        bool from_json(const json& node, std::string_view name, auto& out, std::string& err);

        bool from_json_leaf(const json& node, std::string_view name, source_address& out, std::string& err)
        {
            if (!node.is_string()) { err = std::format("'{}' must be a string", name); return false; }
            const auto text = node.get<std::string>();
            const auto parsed = source_address::try_parse(text);
            if (!parsed.has_value()) {
                err = std::format("'{}' must be k4a:<index>, vz:<index> or a recording path, not '{}'", name, text);
                return false;
            }
            out = *parsed;
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name, pose::view_plane_t& out, std::string& err)
        {
            if (!node.is_string()) { err = std::format("'{}' must be a string", name); return false; }
            const auto text = node.get<std::string>();
            const auto parsed = pose::view_plane_from_name(text);
            if (!parsed.has_value()) {
                err = std::format("'{}' must be frontal or sagittal, not '{}'", name, text);
                return false;
            }
            out = *parsed;
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name,
            pose::tag_detector::pose_method_t& out, std::string& err)
        {
            if (!node.is_string()) { err = std::format("'{}' must be a string", name); return false; }
            const auto text = node.get<std::string>();
            const auto parsed = pose::pose_method_from_str(text);
            if (!parsed.has_value()) {
                err = std::format("'{}' must be orthogonal_iteration or homography, not '{}'", name, text);
                return false;
            }
            out = *parsed;
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name, pose::joint_side_t& out, std::string& err)
        {
            if (!node.is_string()) { err = std::format("'{}' must be a string", name); return false; }
            const auto text = node.get<std::string>();
            const auto parsed = pose::joint_side_from_name(text);
            if (!parsed.has_value()) {
                err = std::format("'{}' must be left, right or midline, not '{}'", name, text);
                return false;
            }
            out = *parsed;
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name, marker_kind_t& out, std::string& err)
        {
            if (!node.is_string()) { err = std::format("'{}' must be a string", name); return false; }
            const auto text = node.get<std::string>();
            const auto parsed = marker_kind_from_name(text);
            if (!parsed.has_value()) {
                err = std::format("'{}' must be apriltag or color_marker, not '{}'", name, text);
                return false;
            }
            out = *parsed;
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name, Eigen::Vector3d& out, std::string& err)
        {
            if (!node.is_array() || node.size() != 3) {
                err = std::format("'{}' must be an array of three numbers", name);
                return false;
            }
            try {
                out = Eigen::Vector3d{ node[0].get<double>(), node[1].get<double>(), node[2].get<double>() };
            }
            catch (const std::exception&) {
                err = std::format("'{}' holds something other than numbers", name);
                return false;
            }
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name, Eigen::Vector2d& out, std::string& err)
        {
            if (!node.is_array() || node.size() != 2) {
                err = std::format("'{}' must be an array of two numbers", name);
                return false;
            }
            try {
                out = Eigen::Vector2d{ node[0].get<double>(), node[1].get<double>() };
            }
            catch (const std::exception&) {
                err = std::format("'{}' holds something other than numbers", name);
                return false;
            }
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name, Eigen::Vector2i& out, std::string& err)
        {
            if (!node.is_array() || node.size() != 2
                || !node[0].is_number_integer() || !node[1].is_number_integer())
            {
                err = std::format("'{}' must be an array of two whole numbers", name);
                return false;
            }
            out = Eigen::Vector2i{ node[0].get<int>(), node[1].get<int>() };
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name, Eigen::Matrix2d& out, std::string& err)
        {
            if (!node.is_array() || node.size() != 2
                || !node[0].is_array() || node[0].size() != 2
                || !node[1].is_array() || node[1].size() != 2)
            {
                err = std::format("'{}' must be two rows of two numbers", name);
                return false;
            }
            try {
                out << node[0][0].get<double>(), node[0][1].get<double>(),
                       node[1][0].get<double>(), node[1][1].get<double>();
            }
            catch (const std::exception&) {
                err = std::format("'{}' holds something other than numbers", name);
                return false;
            }
            return true;
        }

        // Reads `node` into `out`. `name` is the full dotted key, which every message here is
        // built around. An object is filled field by field, so a failure part-way leaves `out`
        // half-written; callers read into a scratch value and keep it only on success.
        bool from_json(const json& node, std::string_view name, auto& out, std::string& err)
        {
            using T = std::remove_cvref_t<decltype(out)>;

            if constexpr (is_optional<T>::value) {
                if (node.is_null()) { out.reset(); return true; }
                typename T::value_type value{};
                if (!from_json(node, name, value, err)) { return false; }
                out = std::move(value);
                return true;
            }
            else if constexpr (is_duration<T>::value) {
                if (!node.is_number()) { err = std::format("'{}' must be a number", name); return false; }
                const double value = node.get<double>();
                if (value < 0.0) { err = std::format("'{}' must not be negative", name); return false; }
                out = T{ value };
                return true;
            }
            else if constexpr (utils::has_serializable_fields<T>) {
                if (!node.is_object()) { err = std::format("'{}' must be an object", name); return false; }
                bool ok = true;
                auto reader = [&](const char* key, auto& field) {
                    if (!ok) { return; }
                    // Every key of the field list has to be there. A default standing in for one
                    // the author left out is the failure this whole reader exists to prevent, and
                    // `--dump-config` prints a complete file to start from.
                    const auto it = node.find(key);
                    if (it == node.end()) {
                        err = std::format("'{}' is missing", key_name(name, key));
                        ok = false;
                        return;
                    }
                    // A field a list cannot write into is a stamp the file states and has to match,
                    // so it is read into a scratch value and compared. The load below is in the
                    // other arm because it would not compile against a const field.
                    using field_t = std::remove_reference_t<decltype(field)>;
                    if constexpr (std::is_const_v<field_t>) {
                        std::remove_const_t<field_t> stated{};
                        if (!from_json(*it, key_name(name, key), stated, err)) { ok = false; return; }
                        if (stated != field) {
                            err = std::format("'{}' is {}, and this build reads {}",
                                key_name(name, key), stated, field);
                            ok = false;
                        }
                    } else {
                        ok = from_json(*it, key_name(name, key), field, err);
                    }
                };
                T::visit_serializable_fields(reader, out);
                return ok; // keys the field list does not name are left alone
            }
            else if constexpr (requires { from_json_leaf(node, name, out, err); }) {
                return from_json_leaf(node, name, out, err);
            }
            else if constexpr (std::integral<T> && !std::same_as<T, bool>) {
                // Read wide and range-check: a number too big for the field would otherwise be
                // truncated into a plausible wrong value.
                if (!node.is_number_integer()) {
                    err = std::format("'{}' must be a whole number", name);
                    return false;
                }
                const int64_t value = node.get<int64_t>();
                if (value < static_cast<int64_t>(std::numeric_limits<T>::min())
                    || value > static_cast<int64_t>(std::numeric_limits<T>::max()))
                {
                    err = std::format("'{}' must be between {} and {}", name,
                        std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
                    return false;
                }
                out = static_cast<T>(value);
                return true;
            }
            else {
                try {
                    out = node.get<T>();
                    return true;
                }
                catch (const std::exception&) {
                    err = std::format("'{}' is not of the expected type", name);
                    return false;
                }
            }
        }

        // =====================================================================================
        // Semantic checks
        //
        // What the types alone cannot say. Reading happens first and answers "is this the right
        // shape"; this answers "is this a usable installation".
        // =====================================================================================

        bool validate(const app_config_t& cfg, std::string& err)
        {
            if (cfg.pose.tag_size_m <= 0.0) {
                err = "'pose.tag_size_m' must be greater than zero";
                return false;
            }
            if (cfg.server.port == 0) {
                err = "'server.port' must not be zero";
                return false;
            }
            if (cfg.camera.roi.has_value() && cfg.camera.roi->is_empty()) {
                err = "'camera.roi' has no area";
                return false;
            }
            if (cfg.pose.detector.apriltag.quad_decimate < 1.0f) {
                err = "'pose.detector.apriltag.quad_decimate' must be at least 1.0 (1.0 = full resolution)";
                return false;
            }
            if (cfg.pose.detector.apriltag.num_iters < 1 || cfg.pose.detector.apriltag.num_threads < 1) {
                err = "'pose.detector.apriltag.num_iters' and '...num_threads' must be at least 1";
                return false;
            }
            if (cfg.pose.frontal.hinge_axis_world.norm() <= 0.0) {
                err = "'pose.frontal.hinge_axis' has zero length";
                return false;
            }

            // Colour markers are read off the image plane, which is the sagittal estimator's input.
            // A frontal run needs a marker whose distance can be solved, and a plain disc is not one.
            if (cfg.pose.detector.kind == marker_kind_t::color_marker
                && cfg.pose.view_plane != pose::view_plane_t::sagittal)
            {
                err = "'pose.detector.kind' color_marker requires 'pose.view_plane' sagittal";
                return false;
            }

            // A colour model is a fixed pair of numbers on the a*b* plane, and a camera free to
            // choose its own exposure or gain moves a marker off them: both scale luminance, and
            // a* and b* follow its cube root. Left on auto that happens mid-run, with nothing to
            // signal it but markers that stop being found, so the pairing is refused up front.
            if (cfg.pose.detector.kind == marker_kind_t::color_marker
                && (!cfg.camera.exposure_us.has_value() || !cfg.camera.gain.has_value()))
            {
                err = "'pose.detector.kind' color_marker requires 'camera.exposure_us' and "
                      "'camera.gain' to name values (a colour model cannot follow auto)";
                return false;
            }

            const color_marker_config_t& color_marker = cfg.pose.detector.color_marker;

            // An absent calibration is not an error: it is what a profile says before anyone has
            // measured this installation. The detector then finds nothing and says so. One that is
            // present has to describe something, since every number in it is a ratio of one
            // measured quantity to another and each has a range it cannot mean anything outside of.
            if (color_marker.calibration.has_value())
            {
                const pose::color_marker_detector::options_t& detector = color_marker.calibration->detector;
                const pose::color_model_t& model = detector.model;
                const Eigen::Matrix2d& covariance = model.cov_ab;

                // A fitted colour is an ellipse on the a*b* plane, so its covariance has to describe
                // an area. A singular one inverts to nonsense and would admit every pixel or none.
                if (covariance(0, 0) <= 0.0 || covariance(1, 1) <= 0.0 || covariance.determinant() <= 0.0) {
                    err = "'pose.detector.color_marker.calibration.detector.model.cov_ab' does not "
                          "describe a spread (both diagonals and the determinant must be positive)";
                    return false;
                }
                if (model.max_distance <= 0.0) {
                    err = "'pose.detector.color_marker.calibration.detector.model.max_distance' "
                          "must be greater than zero";
                    return false;
                }
                if (detector.min_fill <= 0.0 || detector.min_fill > 1.0) {
                    err = "'pose.detector.color_marker.calibration.detector.min_fill' must be within (0, 1]";
                    return false;
                }
                if (detector.min_score < 0.0 || detector.min_score > 1.0) {
                    err = "'pose.detector.color_marker.calibration.detector.min_score' must be within [0, 1]";
                    return false;
                }
                if (detector.max_aspect < 1.0) {
                    err = "'pose.detector.color_marker.calibration.detector.max_aspect' must be at "
                          "least 1.0 (1.0 = a circle)";
                    return false;
                }
            }

            if (color_marker.assigner.leg == pose::joint_side_t::midline) {
                err = "'pose.detector.color_marker.assigner.leg' must name a leg (left or right)";
                return false;
            }
            if (color_marker.assigner.marker_diameter_m <= 0.0) {
                err = "'pose.detector.color_marker.assigner.marker_diameter_m' must be greater than zero";
                return false;
            }
            if (color_marker.assigner.search_radius_px <= 0.0) {
                err = "'pose.detector.color_marker.assigner.search_radius_px' must be greater than zero";
                return false;
            }
            if (color_marker.assigner.bone_length_tolerance <= 0.0 || color_marker.assigner.bone_length_tolerance >= 1.0) {
                err = "'pose.detector.color_marker.assigner.bone_length_tolerance' must be within (0, 1)";
                return false;
            }
            return true;
        }

    } // namespace

    std::filesystem::path project_dir(std::string_view name)
    {
        if (const auto found = search_above_exe(std::filesystem::path{ name })) {
            return found->lexically_normal();
        }

        // No such folder above the executable, so its own directory stands in and output lands flat beside the binary.
        const std::filesystem::path dir = exe_dir();
        return dir.empty() ? std::filesystem::current_path() : dir;
    }

    bool load_config(const std::filesystem::path& path, app_config_t& out, std::string& err)
    {
        std::ifstream in{ path };
        if (!in) {
            err = std::format("cannot open '{}'", path.string());
            return false;
        }

        json root;
        try {
            root = json::parse(in, nullptr /*cb*/, true /*allow_exceptions*/, true /*ignore_comments*/);
        }
        catch (const std::exception& e) {
            err = e.what(); // nlohmann names the byte offset and what it expected there
            return false;
        }

        if (!root.is_object()) {
            err = "the document must be an object";
            return false;
        }

        // Built on a fresh value, so a failure part-way leaves the caller's config untouched.
        app_config_t cfg;
        if (!from_json(root, "", cfg, err)) { return false; }
        if (!validate(cfg, err)) { return false; }

        // A colour block that parsed and passed validation is a colour that was fitted, which is
        // what `valid` means to the detector. The file does not say it: a flag beside the numbers
        // could claim otherwise about them.
        if (auto& calibration = cfg.pose.detector.color_marker.calibration) {
            calibration->detector.model.valid = true;
        }

        // A companion file is named relative to the profile that names it, so a profile folder can
        // be moved or copied whole.
        if (!cfg.camera.intrinsics_file.empty()) {
            const std::filesystem::path p{ cfg.camera.intrinsics_file };
            cfg.camera.intrinsics_file =
                (p.is_absolute() ? p : path.parent_path() / p).lexically_normal().string();
        }

        out = std::move(cfg);
        return true;
    }

    std::string dump_config(const app_config_t& config)
    {
        return to_json(config).dump(2);
    }

    bool save_config(const app_config_t& config, const std::filesystem::path& path, std::string& err)
    {
        std::error_code ec;
        if (path.has_parent_path()) { std::filesystem::create_directories(path.parent_path(), ec); }

        std::ofstream out{ path, std::ios::trunc };
        if (!out) {
            err = std::format("cannot write '{}'", path.string());
            return false;
        }

        out << dump_config(config) << '\n';
        if (!out) {
            err = std::format("failed while writing '{}'", path.string());
            return false;
        }
        return true;
    }

    std::filesystem::path find_config_file(std::string_view name)
    {
        const std::filesystem::path spelled{ name };
        const std::filesystem::path path = spelled.has_parent_path()
            ? spelled
            : project_dir("configs") / spelled;

        std::error_code ec;
        return std::filesystem::exists(path, ec) ? path : std::filesystem::path{};
    }

} // namespace app
