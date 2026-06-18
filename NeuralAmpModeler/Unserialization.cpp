// Unserialization
//
// This plugin is used in important places, so we need to be considerate when
// attempting to unserialize. If the project was last saved with a legacy
// version, then we need it to "update" to the current version is as
// reasonable a way as possible.
//
// In order to handle older versions, the pattern is:
// 1. Implement unserialization for every version into a version-specific
//    struct (Let's use our friend nlohmann::json. Why not?)
// 2. Implement an "update" from each struct to the next one.
// 3. Implement assigning the data contained in the current struct to the
//    current plugin configuration.
//
// This way, a constant amount of effort is required every time the
// serialization changes instead of having to implement a current
// unserialization for each past version.

// Add new unserialization versions to the top, then add logic to the class method at the bottom.

// Boilerplate

void NeuralAmpModeler::_UnserializeApplyConfig(nlohmann::json& config)
{
  auto getParamByName = [&](std::string& name) {
    // Could use a map but eh
    for (int i = 0; i < kNumParams; i++)
    {
      iplug::IParam* param = GetParam(i);
      if (strcmp(param->GetName(), name.c_str()) == 0)
      {
        return param;
      }
    }
    // else
    return (iplug::IParam*)nullptr;
  };
  TRACE
  ENTER_PARAMS_MUTEX
  for (auto it = config.begin(); it != config.end(); ++it)
  {
    std::string name = it.key();
    iplug::IParam* pParam = getParamByName(name);
    if (pParam != nullptr)
    {
      pParam->Set(*it);
      iplug::Trace(TRACELOC, "%s %f", pParam->GetName(), pParam->Value());
    }
    else
    {
      iplug::Trace(TRACELOC, "%s NOT-FOUND", name.c_str());
    }
  }
  OnParamReset(iplug::EParamSource::kPresetRecall);
  LEAVE_PARAMS_MUTEX

  mNAMPath.Set(static_cast<std::string>(config["NAMPath"]).c_str());
  mIRPath.Set(static_cast<std::string>(config["IRPath"]).c_str());

  if (mNAMPath.GetLength())
  {
    _StageModel(mNAMPath);
    if (config.contains("ParametricValues") && config["ParametricValues"].is_array())
    {
      std::vector<RestoredParametricValue> restoredValues;
      restoredValues.reserve(config["ParametricValues"].size());
      for (const auto& entry : config["ParametricValues"])
      {
        if (!entry.is_object() || !entry.contains("name") || !entry.contains("value") || !entry["name"].is_string()
            || !entry["value"].is_number())
          continue;

        RestoredParametricValue restoredValue;
        restoredValue.name = entry["name"].get<std::string>();
        restoredValue.value = entry["value"].get<float>();
        if (entry.contains("index") && entry["index"].is_number_integer())
          restoredValue.index = entry["index"].get<int32_t>();
        restoredValues.push_back(std::move(restoredValue));
      }
      _ApplyRestoredParametricValuesToStagedState(restoredValues);
    }
  }
  if (mIRPath.GetLength())
  {
    _StageIR(mIRPath);
  }
}

bool _TryReadParametricStateExtension(const iplug::IByteChunk& chunk, int startPos, nlohmann::json& config, int& endPos)
{
  endPos = startPos;
  if (startPos < 0 || startPos >= chunk.Size())
    return false;

  WDL_String marker;
  int pos = chunk.GetStr(marker, startPos);
  if (pos < 0 || strcmp(marker.Get(), kParametricStateChunkMagic) != 0)
    return false;

  uint32_t payloadVersion = 0;
  pos = chunk.Get(&payloadVersion, pos);
  if (pos < 0 || payloadVersion != kParametricStateChunkVersion)
    return false;

  int32_t payloadSize = 0;
  pos = chunk.Get(&payloadSize, pos);
  if (pos < 0 || payloadSize < 0)
    return false;

  // The extension is self-delimiting so future payload revisions can be skipped without
  // desynchronizing any later state data that may follow in the same chunk stream.
  const int payloadEndPos = pos + payloadSize;
  if (payloadEndPos < pos || payloadEndPos > chunk.Size())
    return false;

  endPos = payloadEndPos;

  int32_t numValues = 0;
  pos = chunk.Get(&numValues, pos);
  if (pos < 0 || numValues < 0)
    return false;

  auto values = nlohmann::json::array();
  for (int32_t i = 0; i < numValues; ++i)
  {
    int32_t savedIndex = -1;
    WDL_String paramName;
    float value = 0.0f;
    pos = chunk.Get(&savedIndex, pos);
    pos = chunk.GetStr(paramName, pos);
    pos = chunk.Get(&value, pos);
    if (pos < 0)
      return false;
    values.push_back({{"index", savedIndex}, {"name", std::string(paramName.Get())}, {"value", value}});
  }

  if (pos != payloadEndPos)
    return false;

  config["ParametricValues"] = std::move(values);
  return true;
}

// Unserialize NAM Path, IR path, then named keys
int _UnserializePathsAndExpectedKeys(const iplug::IByteChunk& chunk, int startPos, nlohmann::json& config,
                                     std::vector<std::string>& paramNames)
{
  int pos = startPos;
  WDL_String path;
  pos = chunk.GetStr(path, pos);
  config["NAMPath"] = std::string(path.Get());
  pos = chunk.GetStr(path, pos);
  config["IRPath"] = std::string(path.Get());

  for (auto it = paramNames.begin(); it != paramNames.end(); ++it)
  {
    double v = 0.0;
    pos = chunk.Get(&v, pos);
    config[*it] = v;
  }
  return pos;
}

void _RenameKeys(nlohmann::json& j, std::unordered_map<std::string, std::string> newNames)
{
  // Assumes no aliasing!
  for (auto it = newNames.begin(); it != newNames.end(); ++it)
  {
    j[it->second] = j[it->first];
    j.erase(it->first);
  }
}

// v0.7.14

void _UpdateConfigFrom_0_7_14(nlohmann::json& config)
{
  // Fill me in once something changes!
}

int _GetConfigFrom_0_7_14(const iplug::IByteChunk& chunk, int startPos, nlohmann::json& config)
{
  std::vector<std::string> paramNames{"Input",
                                      "Threshold",
                                      "Bass",
                                      "Middle",
                                      "Treble",
                                      "Output",
                                      "NoiseGateActive",
                                      "ToneStack",
                                      "IRToggle",
                                      "CalibrateInput",
                                      "InputCalibrationLevel",
                                      "OutputMode",
                                      "Slim"};

  int pos = _UnserializePathsAndExpectedKeys(chunk, startPos, config, paramNames);
  _UpdateConfigFrom_0_7_14(config);
  return pos;
}

// v0.7.12

void _UpdateConfigFrom_0_7_12(nlohmann::json& config)
{
  config["Slim"] = 1.0;
  _UpdateConfigFrom_0_7_14(config);
}

int _GetConfigFrom_0_7_12(const iplug::IByteChunk& chunk, int startPos, nlohmann::json& config)
{
  std::vector<std::string> paramNames{"Input",
                                      "Threshold",
                                      "Bass",
                                      "Middle",
                                      "Treble",
                                      "Output",
                                      "NoiseGateActive",
                                      "ToneStack",
                                      "IRToggle",
                                      "CalibrateInput",
                                      "InputCalibrationLevel",
                                      "OutputMode"};

  int pos = _UnserializePathsAndExpectedKeys(chunk, startPos, config, paramNames);
  // Then update:
  _UpdateConfigFrom_0_7_12(config);
  return pos;
}

// 0.7.10

void _UpdateConfigFrom_0_7_10(nlohmann::json& config)
{
  // Note: "OutNorm" is Bool-like in v0.7.10, but "OutputMode" is enum.
  // This works because 0 is "Raw" (cf OutNorm false) and 1 is "Calibrated" (cf OutNorm true).
  std::unordered_map<std::string, std::string> newNames{{"OutNorm", "OutputMode"}};
  _RenameKeys(config, newNames);
  // There are new parameters. If they're not included, then 0.7.12 is ok, but future ones might not be.
  config[kCalibrateInputParamName] = (double)kDefaultCalibrateInput;
  config[kInputCalibrationLevelParamName] = kDefaultInputCalibrationLevel;
  _UpdateConfigFrom_0_7_12(config);
}

int _GetConfigFrom_0_7_10(const iplug::IByteChunk& chunk, int startPos, nlohmann::json& config)
{
  std::vector<std::string> paramNames{
    "Input", "Threshold", "Bass", "Middle", "Treble", "Output", "NoiseGateActive", "ToneStack", "OutNorm", "IRToggle"};
  int pos = _UnserializePathsAndExpectedKeys(chunk, startPos, config, paramNames);
  // Then update:
  _UpdateConfigFrom_0_7_10(config);
  return pos;
}

// Earlier than 0.7.10 (Assumed to be 0.7.3-0.7.9)

void _UpdateConfigFrom_Earlier(nlohmann::json& config)
{
  std::unordered_map<std::string, std::string> newNames{{"Gate", "Threshold"}};
  _RenameKeys(config, newNames);
  _UpdateConfigFrom_0_7_10(config);
}

int _GetConfigFrom_Earlier(const iplug::IByteChunk& chunk, int startPos, nlohmann::json& config)
{
  std::vector<std::string> paramNames{
    "Input", "Gate", "Bass", "Middle", "Treble", "Output", "NoiseGateActive", "ToneStack", "OutNorm", "IRToggle"};

  int pos = _UnserializePathsAndExpectedKeys(chunk, startPos, config, paramNames);
  // Then update:
  _UpdateConfigFrom_Earlier(config);
  return pos;
}

//==============================================================================

class _Version
{
public:
  _Version(const int major, const int minor, const int patch)
  : mMajor(major)
  , mMinor(minor)
  , mPatch(patch) {};
  _Version(const std::string& versionStr)
  {
    std::istringstream stream(versionStr);
    std::string token;
    std::vector<int> parts;

    // Split the string by "."
    while (std::getline(stream, token, '.'))
    {
      parts.push_back(std::stoi(token)); // Convert to int and store
    }

    // Check if we have exactly 3 parts
    if (parts.size() != 3)
    {
      throw std::invalid_argument("Input string does not contain exactly 3 segments separated by '.'");
    }

    // Assign the parts to the provided int variables
    mMajor = parts[0];
    mMinor = parts[1];
    mPatch = parts[2];
  };

  bool operator>=(const _Version& other) const
  {
    // Compare on major version:
    if (GetMajor() > other.GetMajor())
    {
      return true;
    }
    if (GetMajor() < other.GetMajor())
    {
      return false;
    }
    // Compare on minor
    if (GetMinor() > other.GetMinor())
    {
      return true;
    }
    if (GetMinor() < other.GetMinor())
    {
      return false;
    }
    // Compare on patch
    return GetPatch() >= other.GetPatch();
  };

  int GetMajor() const { return mMajor; };
  int GetMinor() const { return mMinor; };
  int GetPatch() const { return mPatch; };

private:
  int mMajor;
  int mMinor;
  int mPatch;
};

int NeuralAmpModeler::_UnserializeStateWithKnownVersion(const iplug::IByteChunk& chunk, int startPos)
{
  // We already got through the header before calling this.
  int pos = startPos;

  // Get the version
  WDL_String wVersion;
  pos = chunk.GetStr(wVersion, pos);
  std::string versionStr(wVersion.Get());
  _Version version(versionStr);
  // Act accordingly
  nlohmann::json config;
  if (version >= _Version(0, 7, 14))
  {
    pos = _GetConfigFrom_0_7_14(chunk, pos, config);
  }
  else if (version >= _Version(0, 7, 12))
  {
    pos = _GetConfigFrom_0_7_12(chunk, pos, config);
  }
  else if (version >= _Version(0, 7, 10))
  {
    pos = _GetConfigFrom_0_7_10(chunk, pos, config);
  }
  else if (version >= _Version(0, 7, 9))
  {
    pos = _GetConfigFrom_Earlier(chunk, pos, config);
  }
  else
  {
    // You shouldn't be here...
    assert(false);
  }
  if (pos < chunk.Size())
  {
    int extensionEndPos = pos;
    _TryReadParametricStateExtension(chunk, pos, config, extensionEndPos);
    pos = extensionEndPos;
  }
  _UnserializeApplyConfig(config);
  return pos;
}

int NeuralAmpModeler::_UnserializeStateWithUnknownVersion(const iplug::IByteChunk& chunk, int startPos)
{
  nlohmann::json config;
  int pos = _GetConfigFrom_Earlier(chunk, startPos, config);
  if (pos < chunk.Size())
  {
    int extensionEndPos = pos;
    _TryReadParametricStateExtension(chunk, pos, config, extensionEndPos);
    pos = extensionEndPos;
  }
  _UnserializeApplyConfig(config);
  return pos;
}
