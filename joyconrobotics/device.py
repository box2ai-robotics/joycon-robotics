import hid
from .constants import JOYCON_VENDOR_ID, JOYCON_PRODUCT_IDS
from .constants import JOYCON_L_PRODUCT_ID, JOYCON_R_PRODUCT_ID


def get_device_ids(debug=False):
    """
    returns a list of tuples like `(vendor_id, product_id, serial_number)`
    """
    devices = hid.enumerate(0, 0)

    out = []
    for device in devices:
        vendor_id      = device["vendor_id"]
        product_id     = device["product_id"]
        product_string = device["product_string"]
        serial = device.get('serial') or device.get("serial_number")
        
        if vendor_id != JOYCON_VENDOR_ID:
            continue
        if product_id not in JOYCON_PRODUCT_IDS:
            continue
        if not product_string:
            continue
        
        if serial[0:6] != '9c:54:':
            print(f'{serial=}')
            return (0x057A, product_id, serial)
        
        out.append((vendor_id, product_id, serial))

        if debug:
            print(product_string)
            print(f"\tvendor_id  is {vendor_id!r}")
            print(f"\tproduct_id is {product_id!r}")
            print(f"\tserial     is {serial!r}")
        
    return out


def is_id_L(id):
    return id[1] == JOYCON_L_PRODUCT_ID


def get_ids_of_type(lr, **kw):
    """
    returns a list of tuples like `(vendor_id, product_id, serial_number)`

    arg: lr : str : put `R` or `L`
    """
    if lr.lower() == "l":
        product_id = JOYCON_L_PRODUCT_ID
    else:
        product_id = JOYCON_R_PRODUCT_ID
    return [i for i in get_device_ids(**kw) if i[1] == product_id]


def get_R_ids(**kw):
    """returns a list of tuple like `(vendor_id, product_id, serial_number)`"""
    return get_ids_of_type("R", **kw)


def get_L_ids(**kw):
    """returns a list of tuple like `(vendor_id, product_id, serial_number)`"""
    return get_ids_of_type("L", **kw)


def get_R_id(**kw):
    """returns a tuple like `(vendor_id, product_id, serial_number)`"""
    ids = get_R_ids(**kw)
    if not ids:
        return (None, None, None)
    return ids[0]


def get_L_id(**kw):
    """returns a tuple like `(vendor_id, product_id, serial_number)`"""
    ids = get_L_ids(**kw)
    if not ids:
        return (None, None, None)
    return ids[0]
