// localization_amcl のコンポーネント。M0 では版を返すプロセスだけ (コンテナは M2、issue #4)
#include <optional>
#include <string>

#include "juiz/juiz.h"
#include "juiz_idl.h"
#include "localization_amcl/version.h"

static auto localization_amcl_version_manif = ProcessManifest("localization_amcl_version");
std::optional<std::string> localization_amcl_version() { return localization_amcl::version(); }
COMPONENT_PROCESS_FACTORY(localization_amcl_version_manif, localization_amcl_version);

auto component_manifest() {
    return ComponentManifest("localization_amcl").add_process(localization_amcl_version_manifest());
}
DEFINE_COMPONENT_MANIFEST_ENTRY_POINT(component_manifest)
