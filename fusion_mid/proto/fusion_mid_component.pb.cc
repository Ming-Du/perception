// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: fusion_mid_component.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "fusion_mid_component.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace perception {
namespace mid_fusion {

namespace {

const ::google::protobuf::Descriptor* MidFusionInitOptions_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  MidFusionInitOptions_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_fusion_5fmid_5fcomponent_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_fusion_5fmid_5fcomponent_2eproto() {
  protobuf_AddDesc_fusion_5fmid_5fcomponent_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "fusion_mid_component.proto");
  GOOGLE_CHECK(file != NULL);
  MidFusionInitOptions_descriptor_ = file->message_type(0);
  static const int MidFusionInitOptions_offsets_[3] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MidFusionInitOptions, rec_topic_name_lidar_cloud_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MidFusionInitOptions, rec_topic_name_lidar_obstacle_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MidFusionInitOptions, vehicle_type_),
  };
  MidFusionInitOptions_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      MidFusionInitOptions_descriptor_,
      MidFusionInitOptions::default_instance_,
      MidFusionInitOptions_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MidFusionInitOptions, _has_bits_[0]),
      -1,
      -1,
      sizeof(MidFusionInitOptions),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MidFusionInitOptions, _internal_metadata_),
      -1);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_fusion_5fmid_5fcomponent_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      MidFusionInitOptions_descriptor_, &MidFusionInitOptions::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_fusion_5fmid_5fcomponent_2eproto() {
  delete MidFusionInitOptions::default_instance_;
  delete MidFusionInitOptions_reflection_;
}

void protobuf_AddDesc_fusion_5fmid_5fcomponent_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_fusion_5fmid_5fcomponent_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\032fusion_mid_component.proto\022\025perception"
    ".mid_fusion\"w\n\024MidFusionInitOptions\022\"\n\032r"
    "ec_topic_name_lidar_cloud\030\001 \001(\t\022%\n\035rec_t"
    "opic_name_lidar_obstacle\030\002 \001(\t\022\024\n\014vehicl"
    "e_type\030\003 \001(\t", 172);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "fusion_mid_component.proto", &protobuf_RegisterTypes);
  MidFusionInitOptions::default_instance_ = new MidFusionInitOptions();
  MidFusionInitOptions::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_fusion_5fmid_5fcomponent_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_fusion_5fmid_5fcomponent_2eproto {
  StaticDescriptorInitializer_fusion_5fmid_5fcomponent_2eproto() {
    protobuf_AddDesc_fusion_5fmid_5fcomponent_2eproto();
  }
} static_descriptor_initializer_fusion_5fmid_5fcomponent_2eproto_;

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int MidFusionInitOptions::kRecTopicNameLidarCloudFieldNumber;
const int MidFusionInitOptions::kRecTopicNameLidarObstacleFieldNumber;
const int MidFusionInitOptions::kVehicleTypeFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

MidFusionInitOptions::MidFusionInitOptions()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:perception.mid_fusion.MidFusionInitOptions)
}

void MidFusionInitOptions::InitAsDefaultInstance() {
}

MidFusionInitOptions::MidFusionInitOptions(const MidFusionInitOptions& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:perception.mid_fusion.MidFusionInitOptions)
}

void MidFusionInitOptions::SharedCtor() {
  ::google::protobuf::internal::GetEmptyString();
  _cached_size_ = 0;
  rec_topic_name_lidar_cloud_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  rec_topic_name_lidar_obstacle_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  vehicle_type_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

MidFusionInitOptions::~MidFusionInitOptions() {
  // @@protoc_insertion_point(destructor:perception.mid_fusion.MidFusionInitOptions)
  SharedDtor();
}

void MidFusionInitOptions::SharedDtor() {
  rec_topic_name_lidar_cloud_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  rec_topic_name_lidar_obstacle_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  vehicle_type_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (this != default_instance_) {
  }
}

void MidFusionInitOptions::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* MidFusionInitOptions::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return MidFusionInitOptions_descriptor_;
}

const MidFusionInitOptions& MidFusionInitOptions::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_fusion_5fmid_5fcomponent_2eproto();
  return *default_instance_;
}

MidFusionInitOptions* MidFusionInitOptions::default_instance_ = NULL;

MidFusionInitOptions* MidFusionInitOptions::New(::google::protobuf::Arena* arena) const {
  MidFusionInitOptions* n = new MidFusionInitOptions;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void MidFusionInitOptions::Clear() {
// @@protoc_insertion_point(message_clear_start:perception.mid_fusion.MidFusionInitOptions)
  if (_has_bits_[0 / 32] & 7u) {
    if (has_rec_topic_name_lidar_cloud()) {
      rec_topic_name_lidar_cloud_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    }
    if (has_rec_topic_name_lidar_obstacle()) {
      rec_topic_name_lidar_obstacle_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    }
    if (has_vehicle_type()) {
      vehicle_type_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    }
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool MidFusionInitOptions::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:perception.mid_fusion.MidFusionInitOptions)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional string rec_topic_name_lidar_cloud = 1;
      case 1: {
        if (tag == 10) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_rec_topic_name_lidar_cloud()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->rec_topic_name_lidar_cloud().data(), this->rec_topic_name_lidar_cloud().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_cloud");
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(18)) goto parse_rec_topic_name_lidar_obstacle;
        break;
      }

      // optional string rec_topic_name_lidar_obstacle = 2;
      case 2: {
        if (tag == 18) {
         parse_rec_topic_name_lidar_obstacle:
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_rec_topic_name_lidar_obstacle()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->rec_topic_name_lidar_obstacle().data(), this->rec_topic_name_lidar_obstacle().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_obstacle");
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(26)) goto parse_vehicle_type;
        break;
      }

      // optional string vehicle_type = 3;
      case 3: {
        if (tag == 26) {
         parse_vehicle_type:
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_vehicle_type()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->vehicle_type().data(), this->vehicle_type().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "perception.mid_fusion.MidFusionInitOptions.vehicle_type");
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:perception.mid_fusion.MidFusionInitOptions)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:perception.mid_fusion.MidFusionInitOptions)
  return false;
#undef DO_
}

void MidFusionInitOptions::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:perception.mid_fusion.MidFusionInitOptions)
  // optional string rec_topic_name_lidar_cloud = 1;
  if (has_rec_topic_name_lidar_cloud()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->rec_topic_name_lidar_cloud().data(), this->rec_topic_name_lidar_cloud().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_cloud");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->rec_topic_name_lidar_cloud(), output);
  }

  // optional string rec_topic_name_lidar_obstacle = 2;
  if (has_rec_topic_name_lidar_obstacle()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->rec_topic_name_lidar_obstacle().data(), this->rec_topic_name_lidar_obstacle().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_obstacle");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      2, this->rec_topic_name_lidar_obstacle(), output);
  }

  // optional string vehicle_type = 3;
  if (has_vehicle_type()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->vehicle_type().data(), this->vehicle_type().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "perception.mid_fusion.MidFusionInitOptions.vehicle_type");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      3, this->vehicle_type(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:perception.mid_fusion.MidFusionInitOptions)
}

::google::protobuf::uint8* MidFusionInitOptions::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:perception.mid_fusion.MidFusionInitOptions)
  // optional string rec_topic_name_lidar_cloud = 1;
  if (has_rec_topic_name_lidar_cloud()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->rec_topic_name_lidar_cloud().data(), this->rec_topic_name_lidar_cloud().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_cloud");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->rec_topic_name_lidar_cloud(), target);
  }

  // optional string rec_topic_name_lidar_obstacle = 2;
  if (has_rec_topic_name_lidar_obstacle()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->rec_topic_name_lidar_obstacle().data(), this->rec_topic_name_lidar_obstacle().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_obstacle");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        2, this->rec_topic_name_lidar_obstacle(), target);
  }

  // optional string vehicle_type = 3;
  if (has_vehicle_type()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->vehicle_type().data(), this->vehicle_type().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "perception.mid_fusion.MidFusionInitOptions.vehicle_type");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        3, this->vehicle_type(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:perception.mid_fusion.MidFusionInitOptions)
  return target;
}

int MidFusionInitOptions::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:perception.mid_fusion.MidFusionInitOptions)
  int total_size = 0;

  if (_has_bits_[0 / 32] & 7u) {
    // optional string rec_topic_name_lidar_cloud = 1;
    if (has_rec_topic_name_lidar_cloud()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->rec_topic_name_lidar_cloud());
    }

    // optional string rec_topic_name_lidar_obstacle = 2;
    if (has_rec_topic_name_lidar_obstacle()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->rec_topic_name_lidar_obstacle());
    }

    // optional string vehicle_type = 3;
    if (has_vehicle_type()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->vehicle_type());
    }

  }
  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void MidFusionInitOptions::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:perception.mid_fusion.MidFusionInitOptions)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const MidFusionInitOptions* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const MidFusionInitOptions>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:perception.mid_fusion.MidFusionInitOptions)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:perception.mid_fusion.MidFusionInitOptions)
    MergeFrom(*source);
  }
}

void MidFusionInitOptions::MergeFrom(const MidFusionInitOptions& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:perception.mid_fusion.MidFusionInitOptions)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_rec_topic_name_lidar_cloud()) {
      set_has_rec_topic_name_lidar_cloud();
      rec_topic_name_lidar_cloud_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.rec_topic_name_lidar_cloud_);
    }
    if (from.has_rec_topic_name_lidar_obstacle()) {
      set_has_rec_topic_name_lidar_obstacle();
      rec_topic_name_lidar_obstacle_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.rec_topic_name_lidar_obstacle_);
    }
    if (from.has_vehicle_type()) {
      set_has_vehicle_type();
      vehicle_type_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.vehicle_type_);
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->MergeFrom(from.unknown_fields());
  }
}

void MidFusionInitOptions::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:perception.mid_fusion.MidFusionInitOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void MidFusionInitOptions::CopyFrom(const MidFusionInitOptions& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:perception.mid_fusion.MidFusionInitOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MidFusionInitOptions::IsInitialized() const {

  return true;
}

void MidFusionInitOptions::Swap(MidFusionInitOptions* other) {
  if (other == this) return;
  InternalSwap(other);
}
void MidFusionInitOptions::InternalSwap(MidFusionInitOptions* other) {
  rec_topic_name_lidar_cloud_.Swap(&other->rec_topic_name_lidar_cloud_);
  rec_topic_name_lidar_obstacle_.Swap(&other->rec_topic_name_lidar_obstacle_);
  vehicle_type_.Swap(&other->vehicle_type_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata MidFusionInitOptions::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = MidFusionInitOptions_descriptor_;
  metadata.reflection = MidFusionInitOptions_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// MidFusionInitOptions

// optional string rec_topic_name_lidar_cloud = 1;
bool MidFusionInitOptions::has_rec_topic_name_lidar_cloud() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void MidFusionInitOptions::set_has_rec_topic_name_lidar_cloud() {
  _has_bits_[0] |= 0x00000001u;
}
void MidFusionInitOptions::clear_has_rec_topic_name_lidar_cloud() {
  _has_bits_[0] &= ~0x00000001u;
}
void MidFusionInitOptions::clear_rec_topic_name_lidar_cloud() {
  rec_topic_name_lidar_cloud_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_rec_topic_name_lidar_cloud();
}
 const ::std::string& MidFusionInitOptions::rec_topic_name_lidar_cloud() const {
  // @@protoc_insertion_point(field_get:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_cloud)
  return rec_topic_name_lidar_cloud_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void MidFusionInitOptions::set_rec_topic_name_lidar_cloud(const ::std::string& value) {
  set_has_rec_topic_name_lidar_cloud();
  rec_topic_name_lidar_cloud_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_cloud)
}
 void MidFusionInitOptions::set_rec_topic_name_lidar_cloud(const char* value) {
  set_has_rec_topic_name_lidar_cloud();
  rec_topic_name_lidar_cloud_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_cloud)
}
 void MidFusionInitOptions::set_rec_topic_name_lidar_cloud(const char* value, size_t size) {
  set_has_rec_topic_name_lidar_cloud();
  rec_topic_name_lidar_cloud_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_cloud)
}
 ::std::string* MidFusionInitOptions::mutable_rec_topic_name_lidar_cloud() {
  set_has_rec_topic_name_lidar_cloud();
  // @@protoc_insertion_point(field_mutable:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_cloud)
  return rec_topic_name_lidar_cloud_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 ::std::string* MidFusionInitOptions::release_rec_topic_name_lidar_cloud() {
  // @@protoc_insertion_point(field_release:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_cloud)
  clear_has_rec_topic_name_lidar_cloud();
  return rec_topic_name_lidar_cloud_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void MidFusionInitOptions::set_allocated_rec_topic_name_lidar_cloud(::std::string* rec_topic_name_lidar_cloud) {
  if (rec_topic_name_lidar_cloud != NULL) {
    set_has_rec_topic_name_lidar_cloud();
  } else {
    clear_has_rec_topic_name_lidar_cloud();
  }
  rec_topic_name_lidar_cloud_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), rec_topic_name_lidar_cloud);
  // @@protoc_insertion_point(field_set_allocated:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_cloud)
}

// optional string rec_topic_name_lidar_obstacle = 2;
bool MidFusionInitOptions::has_rec_topic_name_lidar_obstacle() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void MidFusionInitOptions::set_has_rec_topic_name_lidar_obstacle() {
  _has_bits_[0] |= 0x00000002u;
}
void MidFusionInitOptions::clear_has_rec_topic_name_lidar_obstacle() {
  _has_bits_[0] &= ~0x00000002u;
}
void MidFusionInitOptions::clear_rec_topic_name_lidar_obstacle() {
  rec_topic_name_lidar_obstacle_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_rec_topic_name_lidar_obstacle();
}
 const ::std::string& MidFusionInitOptions::rec_topic_name_lidar_obstacle() const {
  // @@protoc_insertion_point(field_get:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_obstacle)
  return rec_topic_name_lidar_obstacle_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void MidFusionInitOptions::set_rec_topic_name_lidar_obstacle(const ::std::string& value) {
  set_has_rec_topic_name_lidar_obstacle();
  rec_topic_name_lidar_obstacle_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_obstacle)
}
 void MidFusionInitOptions::set_rec_topic_name_lidar_obstacle(const char* value) {
  set_has_rec_topic_name_lidar_obstacle();
  rec_topic_name_lidar_obstacle_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_obstacle)
}
 void MidFusionInitOptions::set_rec_topic_name_lidar_obstacle(const char* value, size_t size) {
  set_has_rec_topic_name_lidar_obstacle();
  rec_topic_name_lidar_obstacle_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_obstacle)
}
 ::std::string* MidFusionInitOptions::mutable_rec_topic_name_lidar_obstacle() {
  set_has_rec_topic_name_lidar_obstacle();
  // @@protoc_insertion_point(field_mutable:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_obstacle)
  return rec_topic_name_lidar_obstacle_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 ::std::string* MidFusionInitOptions::release_rec_topic_name_lidar_obstacle() {
  // @@protoc_insertion_point(field_release:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_obstacle)
  clear_has_rec_topic_name_lidar_obstacle();
  return rec_topic_name_lidar_obstacle_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void MidFusionInitOptions::set_allocated_rec_topic_name_lidar_obstacle(::std::string* rec_topic_name_lidar_obstacle) {
  if (rec_topic_name_lidar_obstacle != NULL) {
    set_has_rec_topic_name_lidar_obstacle();
  } else {
    clear_has_rec_topic_name_lidar_obstacle();
  }
  rec_topic_name_lidar_obstacle_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), rec_topic_name_lidar_obstacle);
  // @@protoc_insertion_point(field_set_allocated:perception.mid_fusion.MidFusionInitOptions.rec_topic_name_lidar_obstacle)
}

// optional string vehicle_type = 3;
bool MidFusionInitOptions::has_vehicle_type() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void MidFusionInitOptions::set_has_vehicle_type() {
  _has_bits_[0] |= 0x00000004u;
}
void MidFusionInitOptions::clear_has_vehicle_type() {
  _has_bits_[0] &= ~0x00000004u;
}
void MidFusionInitOptions::clear_vehicle_type() {
  vehicle_type_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_vehicle_type();
}
 const ::std::string& MidFusionInitOptions::vehicle_type() const {
  // @@protoc_insertion_point(field_get:perception.mid_fusion.MidFusionInitOptions.vehicle_type)
  return vehicle_type_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void MidFusionInitOptions::set_vehicle_type(const ::std::string& value) {
  set_has_vehicle_type();
  vehicle_type_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:perception.mid_fusion.MidFusionInitOptions.vehicle_type)
}
 void MidFusionInitOptions::set_vehicle_type(const char* value) {
  set_has_vehicle_type();
  vehicle_type_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:perception.mid_fusion.MidFusionInitOptions.vehicle_type)
}
 void MidFusionInitOptions::set_vehicle_type(const char* value, size_t size) {
  set_has_vehicle_type();
  vehicle_type_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:perception.mid_fusion.MidFusionInitOptions.vehicle_type)
}
 ::std::string* MidFusionInitOptions::mutable_vehicle_type() {
  set_has_vehicle_type();
  // @@protoc_insertion_point(field_mutable:perception.mid_fusion.MidFusionInitOptions.vehicle_type)
  return vehicle_type_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 ::std::string* MidFusionInitOptions::release_vehicle_type() {
  // @@protoc_insertion_point(field_release:perception.mid_fusion.MidFusionInitOptions.vehicle_type)
  clear_has_vehicle_type();
  return vehicle_type_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void MidFusionInitOptions::set_allocated_vehicle_type(::std::string* vehicle_type) {
  if (vehicle_type != NULL) {
    set_has_vehicle_type();
  } else {
    clear_has_vehicle_type();
  }
  vehicle_type_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), vehicle_type);
  // @@protoc_insertion_point(field_set_allocated:perception.mid_fusion.MidFusionInitOptions.vehicle_type)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace mid_fusion
}  // namespace perception

// @@protoc_insertion_point(global_scope)
