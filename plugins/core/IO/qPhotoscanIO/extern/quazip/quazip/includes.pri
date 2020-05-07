OBJECTS_DIR = .obj
MOC_DIR = .moc

unix {
    isEmpty(PREFIX): PREFIX=/usr/local
}

win32 {
    isEmpty(PREFIX): warning("PREFIX unspecified, make install won't work")
}
