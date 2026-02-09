#include "inc/ff.h"

#if FF_USE_LFN && FF_CODE_PAGE != 0

/*---------------------------------------------------------------------------/
/  OEM Code to Unicode Conversion Functions
/---------------------------------------------------------------------------*/

WCHAR ff_oem2uni (	/* Returns converted character */
	WCHAR	oem,	/* Character in OEM encoding to be converted */
	WORD	cp		/* Code page for the conversion */
)
{
	WCHAR c = 0;
	const WCHAR *p;
	UINT i, n;

	if (oem < 0x80) {	/* ASCII? */
		c = oem;
	} else {
		p = 0;
		switch (cp) {
#if FF_CODE_PAGE == 437	/* U.S. */
		case 437:
			p = Tbl437; break;
#endif
#if FF_CODE_PAGE == 720	/* Arabic */
		case 720:
			p = Tbl720; break;
#endif
#if FF_CODE_PAGE == 737	/* Greek */
		case 737:
			p = Tbl737; break;
#endif
#if FF_CODE_PAGE == 771	/* KBL */
		case 771:
			p = Tbl771; break;
#endif
#if FF_CODE_PAGE == 775	/* Baltic */
		case 775:
			p = Tbl775; break;
#endif
#if FF_CODE_PAGE == 850	/* Latin 1 */
		case 850:
			p = Tbl850; break;
#endif
#if FF_CODE_PAGE == 852	/* Latin 2 */
		case 852:
			p = Tbl852; break;
#endif
#if FF_CODE_PAGE == 855	/* Cyrillic */
		case 855:
			p = Tbl855; break;
#endif
#if FF_CODE_PAGE == 857	/* Turkish */
		case 857:
			p = Tbl857; break;
#endif
#if FF_CODE_PAGE == 860	/* Portuguese */
		case 860:
			p = Tbl860; break;
#endif
#if FF_CODE_PAGE == 861	/* Icelandic */
		case 861:
			p = Tbl861; break;
#endif
#if FF_CODE_PAGE == 862	/* Hebrew */
		case 862:
			p = Tbl862; break;
#endif
#if FF_CODE_PAGE == 863	/* Canadian French */
		case 863:
			p = Tbl863; break;
#endif
#if FF_CODE_PAGE == 864	/* Arabic */
		case 864:
			p = Tbl864; break;
#endif
#if FF_CODE_PAGE == 865	/* Nordic */
		case 865:
			p = Tbl865; break;
#endif
#if FF_CODE_PAGE == 866	/* Russian */
		case 866:
			p = Tbl866; break;
#endif
#if FF_CODE_PAGE == 869	/* Greek 2 */
		case 869:
			p = Tbl869; break;
#endif
		default:
			break;
		}
		if (p) {
			c = p[oem - 0x80];
		}
	}
	return c;
}


WCHAR ff_uni2oem (	/* Returns converted character */
	DWORD	uni,	/* Character in Unicode to be converted */
	WORD	cp		/* Code page for the conversion */
)
{
	WCHAR oem = 0;
	const WCHAR *p;
	UINT i, n;

	if (uni < 0x80) {	/* ASCII? */
		oem = (WCHAR)uni;
	} else {
		p = 0;
		switch (cp) {
#if FF_CODE_PAGE == 437	/* U.S. */
		case 437:
			p = Tbl437; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 720	/* Arabic */
		case 720:
			p = Tbl720; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 737	/* Greek */
		case 737:
			p = Tbl737; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 771	/* KBL */
		case 771:
			p = Tbl771; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 775	/* Baltic */
		case 775:
			p = Tbl775; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 850	/* Latin 1 */
		case 850:
			p = Tbl850; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 852	/* Latin 2 */
		case 852:
			p = Tbl852; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 855	/* Cyrillic */
		case 855:
			p = Tbl855; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 857	/* Turkish */
		case 857:
			p = Tbl857; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 860	/* Portuguese */
		case 860:
			p = Tbl860; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 861	/* Icelandic */
		case 861:
			p = Tbl861; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 862	/* Hebrew */
		case 862:
			p = Tbl862; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 863	/* Canadian French */
		case 863:
			p = Tbl863; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 864	/* Arabic */
		case 864:
			p = Tbl864; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 865	/* Nordic */
		case 865:
			p = Tbl865; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 866	/* Russian */
		case 866:
			p = Tbl866; n = 0x80; break;
#endif
#if FF_CODE_PAGE == 869	/* Greek 2 */
		case 869:
			p = Tbl869; n = 0x80; break;
#endif
		default:
			break;
		}
		if (p) {
			for (i = 0; i < n; i++) {
				if (p[i] == uni) {
					oem = (WCHAR)(i + 0x80); break;
				}
			}
		}
	}
	return oem;
}


DWORD ff_wtoupper (	/* Returns upper-case character */
	DWORD uni		/* Unicode character to be upper-cased */
)
{
	if (uni >= 'a' && uni <= 'z') {		/* ASCII */
		uni -= 0x20;
	} else {							/* Unicode */
		/* To be filled by user */
	}
	return uni;
}

#endif /* FF_USE_LFN && FF_CODE_PAGE != 0 */