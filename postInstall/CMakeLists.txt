if( APPLE )
    configure_file(libBundleCloudCompare.py.in libBundleCloudCompare.py)
    configure_file(signatureCloudCompare.sh.in signatureCloudCompare.sh @ONLY )
endif()
