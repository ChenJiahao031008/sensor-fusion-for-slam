// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: scan_contexts.proto

// #include "scan_contexts.pb.h"
#include "lidar_localization/models/scan_context_manager/scan_contexts.pb.h"


#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
extern PROTOBUF_INTERNAL_EXPORT_scan_5fcontexts_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ScanContext_scan_5fcontexts_2eproto;
namespace scan_context_io {
class ScanContextDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ScanContext> _instance;
} _ScanContext_default_instance_;
class ScanContextsDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ScanContexts> _instance;
} _ScanContexts_default_instance_;
}  // namespace scan_context_io
static void InitDefaultsscc_info_ScanContext_scan_5fcontexts_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::scan_context_io::_ScanContext_default_instance_;
    new (ptr) ::scan_context_io::ScanContext();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ScanContext_scan_5fcontexts_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_ScanContext_scan_5fcontexts_2eproto}, {}};

static void InitDefaultsscc_info_ScanContexts_scan_5fcontexts_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::scan_context_io::_ScanContexts_default_instance_;
    new (ptr) ::scan_context_io::ScanContexts();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ScanContexts_scan_5fcontexts_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_ScanContexts_scan_5fcontexts_2eproto}, {
      &scc_info_ScanContext_scan_5fcontexts_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_scan_5fcontexts_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_scan_5fcontexts_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_scan_5fcontexts_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_scan_5fcontexts_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::scan_context_io::ScanContext, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::scan_context_io::ScanContext, data_),
  PROTOBUF_FIELD_OFFSET(::scan_context_io::ScanContexts, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::scan_context_io::ScanContexts, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::scan_context_io::ScanContexts, num_rings_),
  PROTOBUF_FIELD_OFFSET(::scan_context_io::ScanContexts, num_sectors_),
  PROTOBUF_FIELD_OFFSET(::scan_context_io::ScanContexts, data_),
  0,
  1,
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::scan_context_io::ScanContext)},
  { 6, 14, sizeof(::scan_context_io::ScanContexts)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::scan_context_io::_ScanContext_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::scan_context_io::_ScanContexts_default_instance_),
};

const char descriptor_table_protodef_scan_5fcontexts_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\023scan_contexts.proto\022\017scan_context_io\"\033"
  "\n\013ScanContext\022\014\n\004data\030\001 \003(\002\"b\n\014ScanConte"
  "xts\022\021\n\tnum_rings\030\001 \002(\005\022\023\n\013num_sectors\030\002 "
  "\002(\005\022*\n\004data\030\003 \003(\0132\034.scan_context_io.Scan"
  "Context"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_scan_5fcontexts_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_scan_5fcontexts_2eproto_sccs[2] = {
  &scc_info_ScanContext_scan_5fcontexts_2eproto.base,
  &scc_info_ScanContexts_scan_5fcontexts_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_scan_5fcontexts_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_scan_5fcontexts_2eproto = {
  false, false, descriptor_table_protodef_scan_5fcontexts_2eproto, "scan_contexts.proto", 167,
  &descriptor_table_scan_5fcontexts_2eproto_once, descriptor_table_scan_5fcontexts_2eproto_sccs, descriptor_table_scan_5fcontexts_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_scan_5fcontexts_2eproto::offsets,
  file_level_metadata_scan_5fcontexts_2eproto, 2, file_level_enum_descriptors_scan_5fcontexts_2eproto, file_level_service_descriptors_scan_5fcontexts_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_scan_5fcontexts_2eproto(&descriptor_table_scan_5fcontexts_2eproto);
namespace scan_context_io {

// ===================================================================

class ScanContext::_Internal {
 public:
};

ScanContext::ScanContext(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena),
  data_(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:scan_context_io.ScanContext)
}
ScanContext::ScanContext(const ScanContext& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      data_(from.data_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:scan_context_io.ScanContext)
}

void ScanContext::SharedCtor() {
}

ScanContext::~ScanContext() {
  // @@protoc_insertion_point(destructor:scan_context_io.ScanContext)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void ScanContext::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
}

void ScanContext::ArenaDtor(void* object) {
  ScanContext* _this = reinterpret_cast< ScanContext* >(object);
  (void)_this;
}
void ScanContext::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void ScanContext::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ScanContext& ScanContext::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ScanContext_scan_5fcontexts_2eproto.base);
  return *internal_default_instance();
}


void ScanContext::Clear() {
// @@protoc_insertion_point(message_clear_start:scan_context_io.ScanContext)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ScanContext::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated float data = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 13)) {
          ptr -= 1;
          do {
            ptr += 1;
            _internal_add_data(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr));
            ptr += sizeof(float);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<13>(ptr));
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedFloatParser(_internal_mutable_data(), ptr, ctx);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* ScanContext::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:scan_context_io.ScanContext)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated float data = 1;
  for (int i = 0, n = this->_internal_data_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(1, this->_internal_data(i), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:scan_context_io.ScanContext)
  return target;
}

size_t ScanContext::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:scan_context_io.ScanContext)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated float data = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_data_size());
    size_t data_size = 4UL * count;
    total_size += 1 *
                  ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(this->_internal_data_size());
    total_size += data_size;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ScanContext::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:scan_context_io.ScanContext)
  GOOGLE_DCHECK_NE(&from, this);
  const ScanContext* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ScanContext>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:scan_context_io.ScanContext)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:scan_context_io.ScanContext)
    MergeFrom(*source);
  }
}

void ScanContext::MergeFrom(const ScanContext& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:scan_context_io.ScanContext)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void ScanContext::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:scan_context_io.ScanContext)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ScanContext::CopyFrom(const ScanContext& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:scan_context_io.ScanContext)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ScanContext::IsInitialized() const {
  return true;
}

void ScanContext::InternalSwap(ScanContext* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ScanContext::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

class ScanContexts::_Internal {
 public:
  using HasBits = decltype(std::declval<ScanContexts>()._has_bits_);
  static void set_has_num_rings(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_num_sectors(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static bool MissingRequiredFields(const HasBits& has_bits) {
    return ((has_bits[0] & 0x00000003) ^ 0x00000003) != 0;
  }
};

ScanContexts::ScanContexts(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena),
  data_(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:scan_context_io.ScanContexts)
}
ScanContexts::ScanContexts(const ScanContexts& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&num_rings_, &from.num_rings_,
    static_cast<size_t>(reinterpret_cast<char*>(&num_sectors_) -
    reinterpret_cast<char*>(&num_rings_)) + sizeof(num_sectors_));
  // @@protoc_insertion_point(copy_constructor:scan_context_io.ScanContexts)
}

void ScanContexts::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ScanContexts_scan_5fcontexts_2eproto.base);
  ::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
      reinterpret_cast<char*>(&num_rings_) - reinterpret_cast<char*>(this)),
      0, static_cast<size_t>(reinterpret_cast<char*>(&num_sectors_) -
      reinterpret_cast<char*>(&num_rings_)) + sizeof(num_sectors_));
}

ScanContexts::~ScanContexts() {
  // @@protoc_insertion_point(destructor:scan_context_io.ScanContexts)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void ScanContexts::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
}

void ScanContexts::ArenaDtor(void* object) {
  ScanContexts* _this = reinterpret_cast< ScanContexts* >(object);
  (void)_this;
}
void ScanContexts::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void ScanContexts::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ScanContexts& ScanContexts::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ScanContexts_scan_5fcontexts_2eproto.base);
  return *internal_default_instance();
}


void ScanContexts::Clear() {
// @@protoc_insertion_point(message_clear_start:scan_context_io.ScanContexts)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    ::memset(&num_rings_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&num_sectors_) -
        reinterpret_cast<char*>(&num_rings_)) + sizeof(num_sectors_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ScanContexts::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // required int32 num_rings = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_num_rings(&has_bits);
          num_rings_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // required int32 num_sectors = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_num_sectors(&has_bits);
          num_sectors_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .scan_context_io.ScanContext data = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<26>(ptr));
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* ScanContexts::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:scan_context_io.ScanContexts)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required int32 num_rings = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(1, this->_internal_num_rings(), target);
  }

  // required int32 num_sectors = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(2, this->_internal_num_sectors(), target);
  }

  // repeated .scan_context_io.ScanContext data = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:scan_context_io.ScanContexts)
  return target;
}

size_t ScanContexts::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:scan_context_io.ScanContexts)
  size_t total_size = 0;

  if (_internal_has_num_rings()) {
    // required int32 num_rings = 1;
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
        this->_internal_num_rings());
  }

  if (_internal_has_num_sectors()) {
    // required int32 num_sectors = 2;
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
        this->_internal_num_sectors());
  }

  return total_size;
}
size_t ScanContexts::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:scan_context_io.ScanContexts)
  size_t total_size = 0;

  if (((_has_bits_[0] & 0x00000003) ^ 0x00000003) == 0) {  // All required fields are present.
    // required int32 num_rings = 1;
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
        this->_internal_num_rings());

    // required int32 num_sectors = 2;
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
        this->_internal_num_sectors());

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .scan_context_io.ScanContext data = 3;
  total_size += 1UL * this->_internal_data_size();
  for (const auto& msg : this->data_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ScanContexts::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:scan_context_io.ScanContexts)
  GOOGLE_DCHECK_NE(&from, this);
  const ScanContexts* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ScanContexts>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:scan_context_io.ScanContexts)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:scan_context_io.ScanContexts)
    MergeFrom(*source);
  }
}

void ScanContexts::MergeFrom(const ScanContexts& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:scan_context_io.ScanContexts)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      num_rings_ = from.num_rings_;
    }
    if (cached_has_bits & 0x00000002u) {
      num_sectors_ = from.num_sectors_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ScanContexts::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:scan_context_io.ScanContexts)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ScanContexts::CopyFrom(const ScanContexts& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:scan_context_io.ScanContexts)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ScanContexts::IsInitialized() const {
  if (_Internal::MissingRequiredFields(_has_bits_)) return false;
  return true;
}

void ScanContexts::InternalSwap(ScanContexts* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(ScanContexts, num_sectors_)
      + sizeof(ScanContexts::num_sectors_)
      - PROTOBUF_FIELD_OFFSET(ScanContexts, num_rings_)>(
          reinterpret_cast<char*>(&num_rings_),
          reinterpret_cast<char*>(&other->num_rings_));
}

::PROTOBUF_NAMESPACE_ID::Metadata ScanContexts::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace scan_context_io
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::scan_context_io::ScanContext* Arena::CreateMaybeMessage< ::scan_context_io::ScanContext >(Arena* arena) {
  return Arena::CreateMessageInternal< ::scan_context_io::ScanContext >(arena);
}
template<> PROTOBUF_NOINLINE ::scan_context_io::ScanContexts* Arena::CreateMaybeMessage< ::scan_context_io::ScanContexts >(Arena* arena) {
  return Arena::CreateMessageInternal< ::scan_context_io::ScanContexts >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
