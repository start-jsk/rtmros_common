#include "stdio.h"
#include "tchar.h"
#include "Linux4Win.h"

/**
 * @brief LinuxのshmgetシステムコールをWindows用にポーティング
 * @return 共有メモリアクセス用ハンドル
 */
shm_key_t shmget_win(int key, size_t size)
{
	TCHAR	key_str[8];

	memset(key_str, 0, sizeof(key_str));
	_stprintf(key_str, _T("0x%04x"), key);
	return CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, size, key_str);
}
