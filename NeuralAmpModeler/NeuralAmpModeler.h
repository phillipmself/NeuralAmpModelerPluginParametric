#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "../AudioDSPTools/dsp/ImpulseResponse.h"
#include "../AudioDSPTools/dsp/NoiseGate.h"
#include "../AudioDSPTools/dsp/dsp.h"
#include "../AudioDSPTools/dsp/wav.h"
#include "../AudioDSPTools/dsp/ResamplingContainer/ResamplingContainer.h"
#include "../NeuralAmpModelerCore/NAM/dsp.h"
#include "../NeuralAmpModelerCore/NAM/parametric_control.h"
#include "../NeuralAmpModelerCore/NAM/slimmable.h"

#include "Colors.h"
#include "ToneStack.h"

#include "IPlug_include_in_plug_hdr.h"
#include "ISender.h"


const int kNumPresets = 1;
// The plugin is mono inside
constexpr size_t kNumChannelsInternal = 1;

class NAMSender : public iplug::IPeakAvgSender<>
{
public:
  NAMSender()
  : iplug::IPeakAvgSender<>(-90.0, true, 5.0f, 1.0f, 300.0f, 500.0f)
  {
  }
};

enum EParams
{
  // These need to be the first ones because I use their indices to place
  // their rects in the GUI.
  kInputLevel = 0,
  kNoiseGateThreshold,
  kToneBass,
  kToneMid,
  kToneTreble,
  kOutputLevel,
  // The rest is fine though.
  kNoiseGateActive,
  kEQActive,
  kIRToggle,
  // Input calibration
  kCalibrateInput,
  kInputCalibrationLevel,
  kOutputMode,
  kSlim,
  kNumParams
};

const int numKnobs = 6;

enum ECtrlTags
{
  kCtrlTagModelFileBrowser = 0,
  kCtrlTagIRFileBrowser,
  kCtrlTagInputMeter,
  kCtrlTagOutputMeter,
  kCtrlTagSettingsBox,
  kCtrlTagOutputMode,
  kCtrlTagCalibrateInput,
  kCtrlTagInputCalibrationLevel,
  kCtrlTagSlimmableIcon,
  kCtrlTagSlimOverlayBackdrop,
  kCtrlTagSlimKnob,
  kCtrlTagParametricButton,
  kCtrlTagParametricBox,
  kNumCtrlTags
};

enum EMsgTags
{
  // These tags are used from UI -> DSP
  kMsgTagClearModel = 0,
  kMsgTagClearIR,
  kMsgTagHighlightColor,
  // The following tags are from DSP -> UI
  kMsgTagLoadFailed,
  kMsgTagLoadedModel,
  kMsgTagLoadedIR,
  kNumMsgTags
};

// Get the sample rate of a NAM model.
// Sometimes, the model doesn't know its own sample rate; this wrapper guesses 48k based on the way that most
// people have used NAM in the past.
double GetNAMSampleRate(const std::unique_ptr<nam::DSP>& model)
{
  // Some models are from when we didn't have sample rate in the model.
  // For those, this wraps with the assumption that they're 48k models, which is probably true.
  const double assumedSampleRate = 48000.0;
  const double reportedEncapsulatedSampleRate = model->GetExpectedSampleRate();
  const double encapsulatedSampleRate =
    reportedEncapsulatedSampleRate <= 0.0 ? assumedSampleRate : reportedEncapsulatedSampleRate;
  return encapsulatedSampleRate;
};

class ResamplingNAM : public nam::DSP
{
public:
  // Resampling wrapper around the NAM models
  ResamplingNAM(std::unique_ptr<nam::DSP> encapsulated, const double expected_sample_rate)
  : nam::DSP(encapsulated->NumInputChannels(), encapsulated->NumOutputChannels(), expected_sample_rate)
  , mEncapsulated(std::move(encapsulated))
  , mResampler(GetNAMSampleRate(mEncapsulated))
  {
    // Assign the encapsulated object's processing function  to this object's member so that the resampler can use it:
    auto ProcessBlockFunc = [&](NAM_SAMPLE** input, NAM_SAMPLE** output, int numFrames) {
      mEncapsulated->process(input, output, numFrames);
    };
    mBlockProcessFunc = ProcessBlockFunc;

    // Get the other information from the encapsulated NAM so that we can tell the outside world about what we're
    // holding.
    if (mEncapsulated->HasLoudness())
    {
      SetLoudness(mEncapsulated->GetLoudness());
    }
    if (mEncapsulated->HasInputLevel())
    {
      SetInputLevel(mEncapsulated->GetInputLevel());
    }
    if (mEncapsulated->HasOutputLevel())
    {
      SetOutputLevel(mEncapsulated->GetOutputLevel());
    }

    // NOTE: prewarm samples doesn't mean anything--we can prewarm the encapsulated model as it likes and be good to
    // go.
    // _prewarm_samples = 0;

    // And be ready
    int maxBlockSize = 2048; // Conservative
    Reset(expected_sample_rate, maxBlockSize);
  };

  ~ResamplingNAM() = default;

  void prewarm() override { mEncapsulated->prewarm(); };

  void process(NAM_SAMPLE** input, NAM_SAMPLE** output, const int num_frames) override
  {
    if (num_frames > mMaxExternalBlockSize)
      // We can afford to be careful
      throw std::runtime_error("More frames were provided than the max expected!");

    if (!NeedToResample())
    {
      mEncapsulated->process(input, output, num_frames);
    }
    else
    {
      mResampler.ProcessBlock(input, output, num_frames, mBlockProcessFunc);
    }
  };

  int GetLatency() const { return NeedToResample() ? mResampler.GetLatency() : 0; };

  void Reset(const double sampleRate, const int maxBlockSize) override
  {
    mExpectedSampleRate = sampleRate;
    mMaxExternalBlockSize = maxBlockSize;
    mResampler.Reset(sampleRate, maxBlockSize);

    // Allocations in the encapsulated model (HACK)
    // Stolen some code from the resampler; it'd be nice to have these exposed as methods? :)
    const double mUpRatio = sampleRate / GetEncapsulatedSampleRate();
    const auto maxEncapsulatedBlockSize = static_cast<int>(std::ceil(static_cast<double>(maxBlockSize) / mUpRatio));
    mEncapsulated->Reset(sampleRate, maxEncapsulatedBlockSize);
  };

  // So that we can let the world know if we're resampling (useful for debugging)
  double GetEncapsulatedSampleRate() const { return GetNAMSampleRate(mEncapsulated); };

  nam::SlimmableModel* GetSlimmableModel() { return dynamic_cast<nam::SlimmableModel*>(mEncapsulated.get()); }
  const nam::SlimmableModel* GetSlimmableModel() const
  {
    return dynamic_cast<const nam::SlimmableModel*>(mEncapsulated.get());
  }

  nam::IParametricControl* GetParametricControl() { return dynamic_cast<nam::IParametricControl*>(mEncapsulated.get()); }
  const nam::IParametricControl* GetParametricControl() const
  {
    return dynamic_cast<const nam::IParametricControl*>(mEncapsulated.get());
  }
  bool HasParametricControls() const { return GetParametricControl() != nullptr; }

private:
  bool NeedToResample() const { return GetExpectedSampleRate() != GetEncapsulatedSampleRate(); };
  // The encapsulated NAM
  std::unique_ptr<nam::DSP> mEncapsulated;

  // The resampling wrapper
  dsp::ResamplingContainer<NAM_SAMPLE, 1, 12> mResampler;

  // Used to check that we don't get too large a block to process.
  int mMaxExternalBlockSize = 0;

  // This function is defined to conform to the interface expected by the iPlug2 resampler.
  std::function<void(NAM_SAMPLE**, NAM_SAMPLE**, int)> mBlockProcessFunc;
};

class NeuralAmpModeler final : public iplug::Plugin
{
public:
  NeuralAmpModeler(const iplug::InstanceInfo& info);
  ~NeuralAmpModeler();

  void ProcessBlock(iplug::sample** inputs, iplug::sample** outputs, int nFrames) override;
  void OnReset() override;
  void OnIdle() override;

  bool SerializeState(iplug::IByteChunk& chunk) const override;
  int UnserializeState(const iplug::IByteChunk& chunk, int startPos) override;
  void OnUIOpen() override;
  bool OnHostRequestingSupportedViewConfiguration(int width, int height) override { return true; }

  void OnParamChange(int paramIdx) override;
  void OnParamChangeUI(int paramIdx, iplug::EParamSource source) override;
  bool OnMessage(int msgTag, int ctrlTag, int dataSize, const void* pData) override;

private:
  // Allocates mInputPointers and mOutputPointers
  void _AllocateIOPointers(const size_t nChans);
  // Moves DSP modules from staging area to the main area.
  // Also deletes DSP modules that are flagged for removal.
  // Exists so that we don't try to use a DSP module that's only
  // partially-instantiated.
  void _ApplyDSPStaging();
  // Deallocates mInputPointers and mOutputPointers
  void _DeallocateIOPointers();
  // Fallback that just copies inputs to outputs if mDSP doesn't hold a model.
  void _FallbackDSP(iplug::sample** inputs, iplug::sample** outputs, const size_t numChannels, const size_t numFrames);
  // Sizes based on mInputArray
  size_t _GetBufferNumChannels() const;
  size_t _GetBufferNumFrames() const;
  void _InitToneStack();
  // Loads a NAM model and stores it to mStagedNAM
  // Returns an empty string on success, or an error message on failure.
  std::string _StageModel(const WDL_String& dspFile);
  // Loads an IR and stores it to mStagedIR.
  // Return status code so that error messages can be relayed if
  // it wasn't successful.
  dsp::wav::LoadReturnCode _StageIR(const WDL_String& irPath);

  bool _HaveModel() const { return this->mModel != nullptr; };
  // Prepare the input & output buffers
  void _PrepareBuffers(const size_t numChannels, const size_t numFrames);
  // Manage pointers
  void _PrepareIOPointers(const size_t nChans);
  // Copy the input buffer to the object, applying input level.
  // :param nChansIn: In from external
  // :param nChansOut: Out to the internal of the DSP routine
  void _ProcessInput(iplug::sample** inputs, const size_t nFrames, const size_t nChansIn, const size_t nChansOut);
  // Copy the output to the output buffer, applying output level.
  // :param nChansIn: In from internal
  // :param nChansOut: Out to external
  void _ProcessOutput(iplug::sample** inputs, iplug::sample** outputs, const size_t nFrames, const size_t nChansIn,
                      const size_t nChansOut);
  // Resetting for models and IRs, called by OnReset
  void _ResetModelAndIR(const double sampleRate, const int maxBlockSize);

  // Plugin-owned shadow state for the currently-staged/loaded parametric model, if any.
  // Populated from ParamSpec defaults in spec order. Live state is swapped only when the
  // staged model is promoted on the audio thread, so model/state transitions stay aligned.
  struct ParametricModelState
  {
    // Ordered copy of the model's ParamSpec metadata (name/min/max/default).
    std::vector<nam::ParamSpec> specs;
    // Last values committed to the DSP (or defaults, if never applied).
    std::vector<float> currentValues;
    // Audio-thread-owned values waiting to be committed to the DSP. Non-audio threads
    // must publish immutable full-state snapshots for later promotion instead of
    // mutating this vector directly.
    std::vector<float> pendingValues;
    // True when pendingValues differs from what's been committed and needs applying.
    bool dirty = false;
  };

  // Double-buffered, seqlock-style replacement for the old single atomic
  // shared_ptr<const vector<float>> swap. Both buffers are pre-sized to numValues
  // up front so neither the UI-thread publish nor the audio-thread consume ever
  // allocates, and the audio thread never has to touch shared_ptr refcounts in
  // its steady-state path (see mParametricValueSnapshotMailboxRaw).
  struct ParametricValueSnapshotMailbox
  {
    std::array<std::vector<float>, 2> snapshots;
    // Index of the buffer holding the most recently published snapshot.
    std::atomic<size_t> publishedIndex = 0;
    // Seqlock counter: odd while a publish is in progress, even when stable.
    // The consumer retries if it observes an odd value or if the value changes
    // between the start and end of its copy (a torn read).
    std::atomic<uint64_t> publishedVersion = 0;
    // Audio thread marks the buffer it is currently copying so the UI thread
    // does not reuse it for the next publish.
    std::atomic<int> readingIndex = -1;
    size_t numValues = 0;
    // Audio-thread only: remembers the last fully-consumed publishedVersion.
    uint64_t consumedVersion = 0;
  };

  // Immutable serialization-side view of a parametric model. Specs never change after
  // construction; the mailbox carries the latest values without SerializeState() having
  // to touch audio-thread-owned live/staged shadow state directly.
  struct ParametricSerializationSource
  {
    std::vector<nam::ParamSpec> specs;
    std::shared_ptr<ParametricValueSnapshotMailbox> mailbox;
  };

  struct RestoredParametricValue
  {
    int32_t index = -1;
    std::string name;
    float value = 0.0f;
  };

  // Parametric model shadow state helpers.
  // Clears the live specs/values/dirty bookkeeping for the promoted model.
  void _ClearParametricState();
  // Builds shadow state from the model's ParamSpec defaults, in spec order.
  ParametricModelState _CreateParametricStateFromModel(const nam::IParametricControl& parametric) const;
  // Allocates and seeds a per-model double-buffer mailbox for full-state snapshot publication.
  std::shared_ptr<ParametricValueSnapshotMailbox>
  _CreateParametricValueSnapshotMailbox(const std::vector<float>& initialValues) const;
  // Binds immutable specs to a pre-seeded mailbox so state saving can read stable
  // metadata while values continue to flow through the mailbox snapshot path.
  std::shared_ptr<ParametricSerializationSource>
  _CreateParametricSerializationSource(const std::vector<nam::ParamSpec>& specs,
                                       const std::vector<float>& initialValues) const;
  // Copies the latest stable UI-published snapshot into values if one is available.
  bool _TryCopyLatestPublishedParametricValues(const ParametricValueSnapshotMailbox& mailbox,
                                               std::vector<float>& values) const;
  // Picks the currently-active immutable serialization source. This is updated to point
  // at staged state during in-flight swaps so saved paths and saved values stay aligned.
  bool _TryGetParametricStateForSerialization(std::vector<nam::ParamSpec>& specs, std::vector<float>& values) const;
  void _SetActiveParametricSerializationSource(const std::shared_ptr<ParametricSerializationSource>& source);
  // Cheap check for whether promoted plugin-owned parametric shadow state is populated.
  bool _HasParametricState() const;
  // After a staged model load during state restore, matches saved values by parameter
  // identity, clamps them to the current spec ranges, and marks them dirty so the audio
  // thread can commit them before the model processes its first block.
  void _ApplyRestoredParametricValuesToStagedState(const std::vector<RestoredParametricValue>& restoredValues);
  // Audio-thread only: consumes the latest published full-state snapshot, if any, into
  // mParametricState.pendingValues and marks it dirty.
  void _ConsumePublishedParametricValueUpdate();
  // Audio-thread only: commits mParametricState.pendingValues to the live model's
  // IParametricControl when dirty, then advances currentValues. Must run after
  // _ApplyDSPStaging() promotes the live model and before mModel->process().
  void _ApplyPendingParametricStateToModel();
  // UI-thread only: publishes an immutable full-state snapshot for later
  // audio-thread consumption. This is the only Step 6 write path from the
  // dynamic parametric overlay back into the live DSP handoff mailbox.
  void _PublishParametricValueUpdateFromUI(const std::shared_ptr<ParametricValueSnapshotMailbox>& mailbox,
                                           const std::vector<float>& values);

  void _SetInputGain();
  void _SetOutputGain();
  void _ApplySlimParamToLoadedNAMs();

  // See: Unserialization.cpp
  void _UnserializeApplyConfig(nlohmann::json& config);
  // 0.7.9 and later
  int _UnserializeStateWithKnownVersion(const iplug::IByteChunk& chunk, int startPos);
  // Hopefully 0.7.3-0.7.8, but no gurantees
  int _UnserializeStateWithUnknownVersion(const iplug::IByteChunk& chunk, int startPos);

  // Update all controls that depend on a model
  void _UpdateControlsFromModel();
  // Opens the parametric overlay shell if the currently-loaded model supports it.
  void _ShowParametricOverlay();
  // Hides the parametric overlay shell immediately.
  void _CloseParametricOverlay();
  // Synchronizes parametric button visibility and overlay state to the currently-loaded model.
  void _SyncParametricUIFromModel();

  // Make sure that the latency is reported correctly.
  void _UpdateLatency();

  // Update level meters
  // Called within ProcessBlock().
  // Assume _ProcessInput() and _ProcessOutput() were run immediately before.
  void _UpdateMeters(iplug::sample** inputPointer, iplug::sample** outputPointer, const size_t nFrames,
                     const size_t nChansIn, const size_t nChansOut);

  // Member data

  // Input arrays to NAM
  std::vector<std::vector<iplug::sample>> mInputArray;
  // Output from NAM
  std::vector<std::vector<iplug::sample>> mOutputArray;
  // Pointer versions
  iplug::sample** mInputPointers = nullptr;
  iplug::sample** mOutputPointers = nullptr;

  // Input and output gain
  double mInputGain = 1.0;
  double mOutputGain = 1.0;

  // Noise gates
  dsp::noise_gate::Trigger mNoiseGateTrigger;
  dsp::noise_gate::Gain mNoiseGateGain;
  // The model actually being used:
  std::unique_ptr<ResamplingNAM> mModel;
  // And the IR
  std::unique_ptr<dsp::ImpulseResponse> mIR;
  // Manages switching what DSP is being used.
  std::unique_ptr<ResamplingNAM> mStagedModel;
  std::unique_ptr<dsp::ImpulseResponse> mStagedIR;
  // Flags to take away the modules at a safe time.
  std::atomic<bool> mShouldRemoveModel = false;
  std::atomic<bool> mShouldRemoveIR = false;

  std::atomic<bool> mNewModelLoadedInDSP = false;
  std::atomic<bool> mModelCleared = false;

  ParametricModelState mParametricState;
  std::unique_ptr<ParametricModelState> mStagedParametricState;
  // Live/staged immutable serialization sources. Each one owns the mailbox used both for
  // UI/control snapshot publication and for state saving.
  std::shared_ptr<ParametricSerializationSource> mParametricSerializationSource;
  std::shared_ptr<ParametricSerializationSource> mStagedParametricSerializationSource;
  // SerializeState() atomically loads this pointer so it never races the audio thread's
  // promotion/reset of live shadow state. During a staged swap, this points at the
  // staged source so the saved model path and saved param values stay aligned.
  std::shared_ptr<const ParametricSerializationSource> mActiveParametricSerializationSource;
  // Audio thread consumes through this raw pointer so steady-state blocks avoid refcount work.
  // Updated with release ordering alongside the promoted serialization source so a
  // freshly-promoted mailbox's pre-seeded buffers are fully visible before the pointer
  // is observed.
  std::atomic<ParametricValueSnapshotMailbox*> mParametricValueSnapshotMailboxRaw = nullptr;

  // Tone stack modules
  std::unique_ptr<dsp::tone_stack::AbstractToneStack> mToneStack;

  // Post-IR filters
  recursive_linear_filter::HighPass mHighPass;
  //  recursive_linear_filter::LowPass mLowPass;

  // Path to model's config.json or model.nam
  WDL_String mNAMPath;
  // Path to IR (.wav file)
  WDL_String mIRPath;

  WDL_String mHighLightColor{PluginColors::NAM_THEMECOLOR.ToColorCode()};

  std::unordered_map<std::string, double> mNAMParams = {{"Input", 0.0}, {"Output", 0.0}};

  NAMSender mInputSender, mOutputSender;
};
