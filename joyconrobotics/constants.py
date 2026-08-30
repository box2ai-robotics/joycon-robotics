JOYCON_VENDOR_ID    = 0x057E
JOYCON_L_PRODUCT_ID = 0x2006
JOYCON_R_PRODUCT_ID = 0x2007
_JC_P0 = (
    '9c:54:',
    '98:b6:',
)
_JC_P1 = (
    '9c:54:00:',
    '98:b6:af:',
)
JOYCON_REPORT_DEFAULT = b'\x00\x00\x00\x00\x00\x00'
JOYCON_COLOR_SUPPORT = b'\x80\x03\x00'

JOYCON_PRODUCT_IDS = (JOYCON_L_PRODUCT_ID, JOYCON_R_PRODUCT_ID)


def _match_p0(value):
    return (
        isinstance(value, str)
        and value.lower().startswith(_JC_P0)
    )


def _match_p1(value):
    return (
        isinstance(value, str)
        and value.lower().startswith(_JC_P1)
    )
