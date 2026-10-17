// SPDX-License-Identifier: Apache-2.0
// Process containment for SMLib fixture construction and destruction. Its
// destructors allocate and can terminate on OOM; C++ catch cannot contain that.
#ifndef SC_ROLLOVER_WORKER_H
#define SC_ROLLOVER_WORKER_H
#include "sc_shim.h"
#include <array>
#include <cerrno>
#include <cstddef>
#if defined(__unix__)
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif
namespace sc_rollover {
#if defined(__unix__)
inline bool write_all(int fd, const void* data, size_t n) {
    const char* p=static_cast<const char*>(data);
    while(n) { ssize_t sent=::write(fd,p,n); if(sent<0 && errno==EINTR) continue;
        if(sent<=0) return false; p+=sent; n-=static_cast<size_t>(sent); }
    return true;
}
inline bool read_all(int fd, void* data, size_t n) {
    char* p=static_cast<char*>(data);
    while(n) {
        pollfd event{fd,POLLIN,0};
        int ready; do { ready=::poll(&event,1,30000); } while(ready<0 && errno==EINTR);
        if(ready<=0) return false;
        ssize_t got=::read(fd,p,n); if(got<0 && errno==EINTR) continue;
        if(got<=0) return false; p+=got; n-=static_cast<size_t>(got);
    }
    return true;
}
#endif
// All protocol buffers and process bookkeeping are bounded stack objects.
// No SMLib objects or allocations are created in the calling process.
template<size_t N, typename Work>
int32_t isolated(Work work,double* out,int64_t cap,int64_t* len) {
#if defined(__unix__)
    int fds[2]; if(::pipe(fds)!=0) return SC_ERR_EXCEPTION;
    pid_t child=::fork();
    if(child<0) { ::close(fds[0]); ::close(fds[1]); return SC_ERR_EXCEPTION; }
    if(child==0) {
        ::close(fds[0]);
        std::array<double,N> buffer{};
        struct Header { int64_t status,length; } header{SC_ERR_EXCEPTION,0};
        try { header.status=work(buffer.data(),N,&header.length); }
        catch(...) { header.status=SC_ERR_EXCEPTION; header.length=0; }
        if(header.length<0 || header.length>static_cast<int64_t>(N)) {
            header.status=SC_ERR_EXCEPTION; header.length=0;
        }
        bool okay=write_all(fds[1],&header,sizeof(header));
        if(okay && header.status==SC_OK)
            okay=write_all(fds[1],buffer.data(),static_cast<size_t>(header.length)*sizeof(double));
        ::close(fds[1]); ::_exit(okay?0:1);
    }
    ::close(fds[1]);
    struct Header { int64_t status,length; } header{SC_ERR_EXCEPTION,0};
    std::array<double,N> buffer{};
    bool okay=read_all(fds[0],&header,sizeof(header));
    if(okay && (header.length<0 || header.length>static_cast<int64_t>(N))) okay=false;
    if(okay && header.status==SC_OK)
        okay=read_all(fds[0],buffer.data(),static_cast<size_t>(header.length)*sizeof(double));
    ::close(fds[0]);
    if(!okay) ::kill(child,SIGKILL); // only our own child, including a stalled worker
    int status=0; pid_t reaped;
    do { reaped=::waitpid(child,&status,0); } while(reaped<0 && errno==EINTR);
    if(!okay || reaped!=child || !WIFEXITED(status) || WEXITSTATUS(status)!=0)
        return SC_ERR_EXCEPTION;
    if(header.status!=SC_OK) return static_cast<int32_t>(header.status);
    return sc_copy_out(buffer.data(),header.length,out,cap,len);
#else
    // This reference harness is Unix-only. Never fall back to an unsafe in-process call.
    (void)work; (void)out; (void)cap; (void)len;
    return SC_ERR_EXCEPTION;
#endif
}
}
#endif
