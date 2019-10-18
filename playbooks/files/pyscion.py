__all__ = ['SCIONException', 'HostInfo', 'FwdPathMeta', 'Interface', 'Path', 'connect',
    'set_log_level', 'init', 'local_address', 'paths', 'listen']

from ctypes import *
from collections import namedtuple
from ipaddress import IPv4Address
from array import array


_lib = cdll.LoadLibrary('./scion_api.so')


class SCIONException(Exception):
    """ A generic error from SCION """
    def __init__(self, error_msg):
        if isinstance(error_msg, bytes):
            error_msg = _cstr_to_str(error_msg)
        super().__init__(error_msg)


def _str_to_cstr(str):
    return str.encode('utf-8')
def _cstr_to_str(cstr):
    return cast(cstr, c_char_p).value.decode('utf-8')

 # ---------------------------------------------------------

class _PathInterface(Structure):
    _fields_ = [('isdAs', c_char_p), 
                ('ifid', c_size_t)]

class _FwdPathMeta(Structure):
    _fields_ = [('fwdPath_length', c_size_t),
                ('fwdPath', POINTER(c_ubyte)),
                ('mtu', c_ushort),
                ('interfaces_length', c_size_t),
                ('interfaces', POINTER(_PathInterface)),
                ('expTime', c_uint)]

class _HostInfo(Structure):
    _fields_ =[('port', c_ushort),
               ('ipv4', c_ubyte *4)]

class _PathReplyEntry(Structure):
    _fields_ = [('path', POINTER(_FwdPathMeta)),
                ('hostInfo', _HostInfo)]


 # ---------------------------------------------------------

HostInfo = namedtuple('HostInfo', ['ipv4', 'port'])
FwdPathMeta = namedtuple('FwdPathMeta', ['mtu', 'exp_time', 'fwd_path', 'interfaces'])
Interface = namedtuple('Interface', ['isd_as', 'if_id'])


class Path:
    def __init__(self, cpath):
        ip = IPv4Address(bytes(cpath.hostInfo.ipv4))
        self.host_info = HostInfo(
            ipv4=IPv4Address(bytes(cpath.hostInfo.ipv4)),
            port=cpath.hostInfo.port
        )
        p = cpath.path.contents
        interfaces = list(
            Interface(
                isd_as=_cstr_to_str(p.interfaces[i].isdAs),
                if_id=int(p.interfaces[i].ifid) )
            for i in range(p.interfaces_length)
        )
        self.path = FwdPathMeta(
            mtu=p.mtu,
            exp_time=p.expTime,
            fwd_path=bytes( (p.fwdPath[i] for i in range(p.fwdPath_length)) ),
            interfaces=interfaces,
        )

    def __repr__(self):
        path = self.path.interfaces
        return 'PathClass(HostInfo: {hostinfo}, FwdPath: {path})'.format(
            hostinfo=str(self.host_info),
            path=path)

    def to_cstruct(self):
        centry = _PathReplyEntry()
        centry.hostInfo.port = self.host_info.port
        centry.hostInfo.ipv4 = (c_ubyte*4).from_buffer_copy(self.host_info.ipv4.packed)

        path = _FwdPathMeta()
        path.mtu = self.path.mtu
        path.expTime = self.path.exp_time
        path.fwdPath_length = len(self.path.fwd_path)
        path.fwdPath = POINTER(c_ubyte)()
        path.fwdPath = cast(self.path.fwd_path, POINTER(c_ubyte))
        path.interfaces_length = len(self.path.interfaces)
        interfaces = (_PathInterface * len(self.path.interfaces))()
        for i in range(len(self.path.interfaces)):
            interfaces[i].ifid = self.path.interfaces[i].if_id
            interfaces[i].isdAs = _str_to_cstr(self.path.interfaces[i].isd_as)
        path.interfaces = POINTER(_PathInterface)()
        path.interfaces = interfaces

        centry.path = POINTER(_FwdPathMeta)()
        centry.path.contents = path
        return centry


def _build_paths(paths, paths_len):
    return [Path(paths[i]) for i in range(paths_len.value)]


class connect:
    def __init__(self, destination, path):
        self.destination = destination
        self.path = path
        if self.path:
            self.fd = _call_connect(self.destination, self.path)
        else:
            self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        if self.fd is not None:
            self.close()

    def write(self, buffer):
        return _call_write(self.fd, buffer)

    def read(self, buffer):
        return _call_read(self.fd, buffer)

    def close(self):
        return _call_close(self.fd)


 # ---------------------------------------------------------


_lib.SetLogLevel.argtypes = [c_int]
def set_log_level(level):
    """ sets the log level in the API. 0 <= 0 <= 2 """
    _lib.SetLogLevel(level)


_lib.Init.restype = c_char_p
def init():
    err = _lib.Init()
    if err != None:
        raise SCIONException(err)


_lib.LocalAddress.argtypes = [POINTER(POINTER(c_char))]
_lib.LocalAddress.restype = c_char_p
def local_address():
    ptr = POINTER(c_char)()
    err = _lib.LocalAddress(byref(ptr))
    if err != None:
        raise SCIONException(err)
    return _cstr_to_str(ptr)


_lib.Paths.argtypes = [POINTER(c_size_t), POINTER(POINTER(_PathReplyEntry)), POINTER(c_char)]
_lib.Paths.restype = c_char_p
_lib.FreePathsMemory.argtypes = [POINTER(_PathReplyEntry), c_size_t]
_lib.FreePathsMemory.restype = c_char_p
def paths(destination):
    paths_n = c_size_t()
    paths = (POINTER(_PathReplyEntry))()
    err = _lib.Paths(byref(paths_n), byref(paths), _str_to_cstr(destination))
    if err != None:
        raise SCIONException(err)
    pypaths = _build_paths(paths, paths_n)
    err = _lib.FreePathsMemory(paths, paths_n)
    if err != None:
        raise SCIONException(err)
    return pypaths


_lib.Connect.argtypes = [POINTER(c_long), POINTER(c_char), POINTER(_PathReplyEntry)]
_lib.Connect.restype = c_char_p
def _call_connect(destination, path):
    fd = c_long()
    cpath = path.to_cstruct()
    err = _lib.Connect(byref(fd), _str_to_cstr(destination), cpath)
    if err != None:
        raise SCIONException(err)
    return int(fd.value)


_lib.Close.argtypes = [c_long]
_lib.Close.restype = c_char_p
def _call_close(fd):
    err = _lib.Close(fd)
    if err != None:
        raise SCIONException(err)


_lib.Write.argtypes = [c_long, POINTER(c_char), c_size_t]
_lib.Write.restype = c_char_p
def _call_write(fd, buffer):
    addr, count = array('B', buffer).buffer_info()
    assert count == len(buffer)
    ptr = cast(addr,POINTER(c_char))
    err = _lib.Write(fd, ptr, len(buffer))
    if err != None:
        raise SCIONException(err)


_lib.Read.argtypes = [POINTER(c_size_t), POINTER(POINTER(c_char)), c_long, POINTER(c_ubyte), c_size_t]
_lib.Read.restype = c_char_p
def _call_read(fd, buffer):
    c_buffer = (c_ubyte * len(buffer)).from_buffer(buffer)
    n = c_size_t()
    client_address = POINTER(c_char)()
    err = _lib.Read(byref(n), byref(client_address), fd, c_buffer, len(buffer))
    if err != None:
        raise SCIONException(err)
    return _cstr_to_str(client_address), int(n.value)


_lib.Listen.argtypes = [POINTER(c_long), c_ushort]
_lib.Listen.restype = c_char_p
def listen(port):
    fd = c_long()
    err = _lib.Listen(byref(fd), c_ushort(port))
    if err != None:
        raise SCIONException(err)
    conn = connect(None, None)
    conn.fd = int(fd.value)
    return conn
