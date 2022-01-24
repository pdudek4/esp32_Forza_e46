idf_component_register(SRCS "main.c"
							"blufi_security.c"
							"blufi_init.c"
					REQUIRES bt
					REQUIRES wpa_supplicant
                    INCLUDE_DIRS ".")
