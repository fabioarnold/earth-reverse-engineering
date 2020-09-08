// https://github.com/marin-m/pbtk/wiki/Protobuf-URL

std::string protoUrlType(google::protobuf::FieldDescriptor::CppType cpp_type) {
	switch (cpp_type) {
		case google::protobuf::FieldDescriptor::CppType::CPPTYPE_INT32: return "i";    // TYPE_INT32, TYPE_SINT32, TYPE_SFIXED32
		case google::protobuf::FieldDescriptor::CppType::CPPTYPE_INT64: return "j";    // TYPE_INT64, TYPE_SINT64, TYPE_SFIXED64
		case google::protobuf::FieldDescriptor::CppType::CPPTYPE_UINT32: return "u";   // TYPE_UINT32, TYPE_FIXED32
		case google::protobuf::FieldDescriptor::CppType::CPPTYPE_UINT64: return "v";   // TYPE_UINT64, TYPE_FIXED64
		case google::protobuf::FieldDescriptor::CppType::CPPTYPE_DOUBLE: return "d";   // TYPE_DOUBLE
		case google::protobuf::FieldDescriptor::CppType::CPPTYPE_FLOAT: return "f";    // TYPE_FLOAT
		case google::protobuf::FieldDescriptor::CppType::CPPTYPE_BOOL: return "b";     // TYPE_BOOL
		case google::protobuf::FieldDescriptor::CppType::CPPTYPE_ENUM: return "e";     // TYPE_ENUM
		case google::protobuf::FieldDescriptor::CppType::CPPTYPE_STRING: return "s";   // TYPE_STRING, TYPE_BYTES
		case google::protobuf::FieldDescriptor::CppType::CPPTYPE_MESSAGE: return "m";  // TYPE_MESSAGE, TYPE_GROUP
		default: assert(false);
	}
	return "ERROR";
}

#ifdef GetMessage
#undef GetMessage // defined by windows.h
#endif
std::string protoUrlEncode(const google::protobuf::Message& msg) {
	std::stringstream url;

	auto descriptor = msg.GetDescriptor();
	auto reflection = msg.GetReflection();
	for (int i = 0; i < descriptor->field_count(); i++) {
		auto fd = descriptor->field(i);
		assert(!fd->is_repeated()); // TODO: handle flag repeated
		url << "!" << fd->number() << protoUrlType(fd->cpp_type());
		switch (fd->cpp_type()) {
			case google::protobuf::FieldDescriptor::CppType::CPPTYPE_INT32:
				url << reflection->GetInt32(msg, fd);
				break;
			case google::protobuf::FieldDescriptor::CppType::CPPTYPE_INT64:
				url << reflection->GetInt64(msg, fd);
				break;
			case google::protobuf::FieldDescriptor::CppType::CPPTYPE_UINT32:
				url << reflection->GetUInt32(msg, fd);
				break;
			case google::protobuf::FieldDescriptor::CppType::CPPTYPE_UINT64:
				url << reflection->GetUInt64(msg, fd);
				break;
			case google::protobuf::FieldDescriptor::CppType::CPPTYPE_DOUBLE:
				url << reflection->GetDouble(msg, fd);
				break;
			case google::protobuf::FieldDescriptor::CppType::CPPTYPE_FLOAT:
				url << reflection->GetFloat(msg, fd);
				break;
			case google::protobuf::FieldDescriptor::CppType::CPPTYPE_BOOL:
				url << reflection->GetBool(msg, fd);
				break;
			case google::protobuf::FieldDescriptor::CppType::CPPTYPE_ENUM:
				url << reflection->GetEnum(msg, fd)->number();
				break;
			case google::protobuf::FieldDescriptor::CppType::CPPTYPE_STRING:
				url << reflection->GetString(msg, fd); // TODO: replace ! and * and quote special chars
				break;
			case google::protobuf::FieldDescriptor::CppType::CPPTYPE_MESSAGE: {
				auto value = protoUrlEncode(reflection->GetMessage(msg, fd, nullptr));
				url << std::count(value.begin(), value.end(), '!');
				url << value;
			} break;
			default: assert(false);
		}
	}

	return url.str();
}
