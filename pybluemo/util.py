
class RgbRle(object):
    ANIM_FIXED = 0
    ANIM_ROTATION = 1
    ANIM_ROT_INDIVIDUAL = 2
    ANIM_FRAME_BY_FRAME = 3

    TYPE_NORMAL = 0
    TYPE_ASCENDING = 1
    TYPE_DESCENDING = 2
    TYPE_UPDOWN = 3
    TYPE_REV_ASC = 4
    TYPE_REV_DESC = 5
    TYPE_REV_UPDOWN = 6

    def __init__(self):
        self.encoding = ""

    def add_run_length(self, offset, length, color, horiz=1, rle_type=TYPE_NORMAL, eof=False):
        if eof:
            eof = 0x80
        else:
            eof = 0x00
        self.encoding += chr((((offset >> 6) & 0x0F) + eof) + (rle_type << 4))
        self.encoding += chr(((offset << 2) & 0xFC) + ((length >> 8) & 0x03))
        self.encoding += chr(length & 0xFF)
        self.encoding += color
        self.encoding += chr(horiz & 0xFF)
        return self

    def get_bytes(self):
        return b"".join(b"%c" % ord(i) for i in self.encoding)
