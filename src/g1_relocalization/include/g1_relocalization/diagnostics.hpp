#pragma once

#include <cerrno>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <openssl/evp.h>

namespace g1_relocalization
{
inline std::string sha256(std::istream & input)
{
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
    EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("Cannot initialize SHA-256");
  }
  char buffer[65536];
  while (input.read(buffer, sizeof(buffer)) || input.gcount()) {
    if (EVP_DigestUpdate(context.get(), buffer, input.gcount()) != 1) {
      throw std::runtime_error("Cannot update SHA-256");
    }
  }
  if (input.bad()) {throw std::runtime_error("Cannot read SHA-256 input");}
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  if (EVP_DigestFinal_ex(context.get(), digest, &length) != 1) {
    throw std::runtime_error("Cannot finish SHA-256");
  }
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < length; ++i) {out << std::setw(2) << unsigned(digest[i]);}
  return out.str();
}

inline std::string fileSha256(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {throw std::runtime_error("Cannot hash file: " + path);}
  return sha256(input);
}

// POSIX APIs keep the node compatible with Foxy's C++14/PCL toolchain.
inline void makeDirectories(const std::string & path)
{
  if (path.empty()) {throw std::invalid_argument("Empty diagnostics directory");}
  for (std::size_t i = 1; i <= path.size(); ++i) {
    if (i != path.size() && path[i] != '/') {continue;}
    const auto part = path.substr(0, i);
    if (::mkdir(part.c_str(), 0750) != 0 && errno != EEXIST) {
      throw std::runtime_error("Cannot create diagnostics directory: " + part);
    }
    struct stat info;
    if (::stat(part.c_str(), &info) != 0 || !S_ISDIR(info.st_mode)) {
      throw std::runtime_error("Diagnostics path is not a directory: " + part);
    }
  }
}
}  // namespace g1_relocalization
