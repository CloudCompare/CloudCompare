#ifndef MEMORY_FILE_H
#define MEMORY_FILE_H

#include <cstring> // for size_t

/* This files comes from http://en.wikibooks.org/wiki/Optimizing_C%2B%2B/General_optimization_techniques/Input/Output
 * The code is released as CC Share Alike license. So it should be ok to introduce that here
 * I am not sure this will work correctly also on win. So please check.
 */

/*
  Read-only memory-mapped file wrapper.
  It handles only files that can be wholly loaded
  into the address space of the process.
  The constructor opens the file, the destructor closes it.
  The "data" function returns a pointer to the beginning of the file,
  if the file has been successfully opened, otherwise it returns 0.
  The "size" function returns the length of the file in bytes,
  if the file has been successfully opened, otherwise it returns 0.
*/
class InputMemoryFile {
public:
    InputMemoryFile(const char *pathname);
    ~InputMemoryFile();
    const char* data() const { return data_; }
    size_t size() const { return size_; }
private:
    const char* data_;
    size_t size_;
#if defined(__unix__)
    int file_handle_;
#elif defined(_WIN32)
    typedef void* HANDLE;
    HANDLE file_handle_;
    HANDLE file_mapping_handle_;
#else
    #error Only Posix or Windows systems can use memory-mapped files.
#endif
};

/*
  Read/write memory-mapped file wrapper.
  It handles only files that can be wholly loaded
  into the address space of the process.
  The constructor opens the file, the destructor closes it.
  The "data" function returns a pointer to the beginning of the file,
  if the file has been successfully opened, otherwise it returns 0.
  The "size" function returns the initial length of the file in bytes,
  if the file has been successfully opened, otherwise it returns 0.
  Afterwards it returns the size the physical file will get if it is closed now.
  The "resize" function changes the number of bytes of the significant
  part of the file. The resulting size can be retrieved
  using the "size" function.
  The "reserve" grows the phisical file to the specified number of bytes.
  The size of the resulting file can be retrieved using "capacity".
  Memory mapped files cannot be shrinked;
  a value smaller than the current capacity is ignored.
  The "capacity()" function return the size the physical file has at this time.
  The "flush" function ensure that the disk is updated
  with the data written in memory.
*/
class MemoryFile {
public:
    enum e_open_mode {
        if_exists_fail_if_not_exists_create,
        if_exists_keep_if_dont_exists_fail,
        if_exists_keep_if_dont_exists_create,
        if_exists_truncate_if_not_exists_fail,
        if_exists_truncate_if_not_exists_create,
    };
    MemoryFile(const char *pathname, e_open_mode open_mode);
    ~MemoryFile();
    char* data() { return data_; }
    void resize(size_t new_size);
    void reserve(size_t new_capacity);
    size_t size() const { return size_; }
    size_t capacity() const { return capacity_; }
    bool flush();
private:
    char* data_;
    size_t size_;
    size_t capacity_;
#if defined(__unix__)
    int file_handle_;
#elif defined(_WIN32)
    typedef void * HANDLE;
    HANDLE file_handle_;
    HANDLE file_mapping_handle_;
#else
    #error Only Posix or Windows systems can use memory-mapped files.
#endif
};
#endif
