#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/types.h>
#include <signal.h>

#include "ak_common.h"
#include "ak_thread.h"
#include "ak_cmd_exec.h"
#include "ak_vi.h"
#include "ak_ai.h"
#include "ak_aenc.h"
#include "ak_rtsp.h"
#include "ak_dvr_record.h"
#include "ak_net.h"

#include <akae_stdlib.h>
#include <akae_rtspserver.h>
#include <akae_thread.h>

/* ===== EXTERNAL AI HANDLE ===== */
extern void *ai_handle;  // declared in ipc_main.c

#define LEN_IFACE 32
#define LEN_IP    32
#define LEN_LINK  128

AK_int ak_rtsp_stop(AK_int index)
{
    (void)index;  /* index unused, silence warnings */

    ak_print_notice("ak_rtsp_stop() called\n");

    ak_rtsp_exit();   /* safe full cleanup */

    return AK_SUCCESS;
}


/**
 * 内部单体使用的内存分配器，初始化代表模块已经初始化。
 */
static AK_Object _Heap = AK_null;

/* Forward declarations with correct AK_ThreadFunc signature */
static AK_void _buffer_aenc(AK_Thread th, AK_int argc, AK_voidptr argv[]);
static AK_void _buffer_venc(AK_Thread th, AK_int argc, AK_voidptr argv[]);




/**
 * @ref _Heap 句柄对应的所分配内存。
 */
static AK_byte _heap [1024 * 1024 * 4];

/**
 * RTSP 服务器单体句柄。
 */
static AK_Object _Server = AK_null;


/// 后台码流缓冲线程句柄。
static AK_Thread _VStreamTH[RTSP_CHANNEL_NUM], _AStreamTH;

/// 媒体缓冲队列句柄。
static AK_Object _VStreamQ[RTSP_CHANNEL_NUM], _711aStreamQ;


/// 传入配置参数。
static struct rtsp_param _rtsp_ctrl_param;
#if 0
static void rtsp_venc_set_bps(
    void *venc_handle,
    struct rtsp_channel_cfg_t *rtsp_chn)
{
    int target_kbps = 0, max_kbps = 0;

    if (rtsp_chn->target_ratio != 0) {
        max_kbps = rtsp_chn->max_kbps;
        target_kbps = rtsp_chn->target_ratio * max_kbps / 100;

        ak_venc_set_kbps(venc_handle, target_kbps, max_kbps);

        if (rtsp_chn->smart.smart_mode != 0) {
            ak_venc_set_smart_config(venc_handle, &rtsp_chn->smart);
        }
    }
}
#endif

static const char rtsp_version[] = "libapp_rtsp V2.0.00";

/*
 * init video encode by index to indicate which encode group
 */
 
/**
 * ak_rtsp_get_version - get rtsp version
 * return: version string
 */
const char* ak_rtsp_get_version(void)
{
	return rtsp_version;
}

static inline int rtsp_ch_to_vi_ch(int rtsp_ch)
{
    /* RTSP ch 0 = main, RTSP ch 1 = sub */
    return rtsp_ch;
}


/* VIDEO-ONLY RTSP - Pas d'audio
 * 
 * Remplacez la fonction _buffer_aenc dans ak_rtsp.c
 */

static AK_void _buffer_aenc(AK_Thread th, AK_int argc, AK_voidptr argv[])
{
    /* Désactiver complètement l'audio RTSP */
    ak_print_notice("RTSP: Audio disabled (video only)\n");
    
    /* Le thread se termine immédiatement */
    return;
}

#if 0
static AK_void _buffer_venc(AK_Thread th, AK_int argc, AK_voidptr argv[])
{
    int ch = (int)(intptr_t)argv[0];
    void *vstream = _rtsp_ctrl_param.rtsp_chn[ch].vstream;  // ← Utiliser la copie globale!

    ak_print_notice_ex("RTSP ch=%d vstream=%p\n", ch, vstream);

    if (!vstream) {
        ak_print_error("vstream NULL for ch=%d\n", ch);
        return;
    }

    // Attendre que le VI soit prêt
    int wait_count = 0;
    while (!akae_thread_terminated(th) && wait_count < 50) {
        struct video_stream vs = {0};
        
        if (AK_SUCCESS == ak_venc_get_stream(vstream, &vs)) {
            ak_print_notice("RTSP ch=%d: VI ready!\n", ch);
            ak_venc_release_stream(vstream, &vs);
            break;
        }
        
        wait_count++;
        if (wait_count % 10 == 0) {
            ak_print_notice("RTSP ch=%d: waiting... (%d/50)\n", ch, wait_count);
        }
        akae_thread_suspend(th, 0, 200, 0);
    }

    if (wait_count >= 50) {
        ak_print_error("RTSP ch=%d: VI timeout\n", ch);
        return;
    }

    ak_print_notice("RTSP ch=%d: streaming!\n", ch);

    // Boucle principale
    while (!akae_thread_terminated(th)) {
        struct video_stream vs = {0};

        while (AK_SUCCESS == ak_venc_get_stream(vstream, &vs)) {
            akae_rtp_queue_add_payload(_VStreamQ[ch], vs.ts, vs.data, vs.len);
            ak_venc_release_stream(vstream, &vs);
        }

        akae_thread_suspend(th, 0, 30, 0);
    }
}
static AK_void _buffer_venc(AK_Thread th, AK_int argc, AK_voidptr argv[])
{
    int ch = (int)(intptr_t)argv[0];
    void *vstream = _rtsp_ctrl_param.rtsp_chn[ch].vstream;

    ak_print_notice_ex("RTSP ch=%d vstream=%p\n", ch, vstream);

    if (!vstream) {
        ak_print_error("vstream NULL for ch=%d\n", ch);
        return;
    }

    // ← ATTENDRE 3 SECONDES avant le premier appel
    ak_print_notice("RTSP ch=%d: waiting 3s for VI...\n", ch);
    akae_thread_suspend(th, 0, 3000, 0);
    ak_print_notice("RTSP ch=%d: starting now\n", ch);

    // Boucle principale
    while (!akae_thread_terminated(th)) {
        struct video_stream vs = {0};
        int ret;

        ret = ak_venc_get_stream(vstream, &vs);
        if (ret == AK_SUCCESS) {
            akae_rtp_queue_add_payload(_VStreamQ[ch], vs.ts, vs.data, vs.len);
            ak_venc_release_stream(vstream, &vs);
        }

        akae_thread_suspend(th, 0, 30, 0);
    }
    
    ak_print_notice("RTSP ch=%d: stopped\n", ch);
}
#endif
static AK_void _buffer_venc(AK_Thread th, AK_int argc, AK_voidptr argv[])
{
    int ch = (int)(intptr_t)argv[0];
    void *vstream = NULL;

    ak_print_notice_ex("RTSP THREAD start ch=%d\n", ch);

    /* 🔑 WAIT until DVR creates vstream */
    while (!akae_thread_terminated(th)) {
        vstream = ak_dvr_record_get_vstream_by_chn(ch);
        if (vstream) {
            ak_print_notice_ex("RTSP ch=%d got vstream=%p\n", ch, vstream);
            break;
        }
        akae_thread_suspend(th, 0, 200, 0);
    }

    if (!vstream) {
        ak_print_error("RTSP ch=%d exit: vstream NULL\n", ch);
        return;
    }

    /* Optional: wait for first frame */
    struct video_stream vs = {0};
    while (!akae_thread_terminated(th)) {
        if (AK_SUCCESS == ak_venc_get_stream(vstream, &vs)) {
            ak_venc_release_stream(vstream, &vs);
            break;
        }
        akae_thread_suspend(th, 0, 200, 0);
    }

    ak_print_notice_ex("RTSP ch=%d streaming\n", ch);

    while (!akae_thread_terminated(th)) {
        while (AK_SUCCESS == ak_venc_get_stream(vstream, &vs)) {
            akae_rtp_queue_add_payload(
                _VStreamQ[ch],
                vs.ts,
                vs.data,
                vs.len
            );
            ak_venc_release_stream(vstream, &vs);
        }
        akae_thread_suspend(th, 0, 30, 0);
    }
}

/**
 * ak_rtsp_init - init rtsp param
 *                start rtsp dispense sever
 *                start rtsp listen\recv\send
 * return: 0 -> success, -1 -> failed
 */
int ak_rtsp_init(struct rtsp_param *param)
{
	char ac_iface[LEN_IFACE], ac_ip[LEN_IP], ac_rtsp[LEN_LINK];
	AK_int argc = 0;
	AK_voidptr argv[32];
	AK_int i = 0;
	AK_int registry = 0;
	AK_int port = 554;

	AK_EXPECT_RETURN_VAL (AK_null == _Heap, AK_FAILED);

	if (!param) {
		ak_print_error_ex("invalid argument\n");
		return -1;
	}
	

ak_print_error("RTSP bind: ch0=%p ch1=%p\n",
    param->rtsp_chn[0].vstream,
    param->rtsp_chn[1].vstream);
	
	
	
/* ---- EARLY VALIDATION: MUST be before Heap/Server creation ---- */
for (i = 0; i < RTSP_CHANNEL_NUM; i++) {
    if (param->rtsp_chn[i].suffix_name[0] == '\0') {
        ak_print_error_ex("RTSP init: ch=%d empty suffix\n", i);
        return -1;
    }

    if (param->rtsp_chn[i].width == 0 || param->rtsp_chn[i].height == 0) {
        ak_print_error_ex(
            "RTSP init too early: ch=%d invalid resolution %ux%u\n",
            i,
            param->rtsp_chn[i].width,
            param->rtsp_chn[i].height
        );
        return -1;
    }

    if (param->rtsp_chn[i].vi_handle == NULL) {
        ak_print_error_ex("RTSP init: ch=%d vi_handle NULL\n", i);
        return -1;
    }
}
	/// 初始化栈分配器
_Heap = akae_malloc_create(AK_true, _heap, sizeof(_heap));
if (_Heap == AK_null) {
    ak_print_error_ex("RTSP: heap create failed\n");
    return -1;
}

memcpy(&_rtsp_ctrl_param, param, sizeof(_rtsp_ctrl_param));

_Server = akae_rtsp_server_create(_Heap);
if (_Server == AK_null) {
    ak_print_error_ex("RTSP: server create failed\n");
    goto fail;
}

akae_rtsp_server_verbose(_Server, AK_true);
akae_rtsp_server_verbose_http(_Server, AK_true);

/// 创建音频缓冲
/*
_711aStreamQ = akae_rtp_queue_create(_Heap, AK_RTP_PT_PCMA);
if (_711aStreamQ == AK_null) {
    ak_print_error_ex("RTSP: audio queue create failed\n");
    goto fail;
}
*/
_AStreamTH = akae_thread_create(
    "aenc",
    0,                      // stack size
    0,                      // priority
    _buffer_aenc,           // ✅ Direct call, correct signature
    0,
    NULL
);

if (_AStreamTH <= 0) {
    ak_print_error_ex("RTSP: audio thread create failed\n");
    goto fail;
}


for (i = 0; i < RTSP_CHANNEL_NUM; ++i) {

    _VStreamQ[i] = akae_rtp_queue_create(
        _Heap,
        HEVC_ENC_TYPE == param->rtsp_chn[i].video_enc_type
            ? AK_RTP_PT_H265
            : AK_RTP_PT_H264
    );
    if (_VStreamQ[i] == AK_null) {
        ak_print_error_ex("RTSP: video queue failed ch=%d\n", i);
        goto fail;
    }

    registry = akae_rtsp_server_register_url(
        _Server,
        param->rtsp_chn[i].suffix_name,
        AK_null,
        AK_null
    );
    if (registry <= 0) {
        ak_print_error_ex(
            "RTSP register url failed ch=%d suffix=%s\n",
            i,
            param->rtsp_chn[i].suffix_name
        );
        goto fail;
    }

    akae_rtsp_server_describe_video(
        _Server,
        registry,
        akae_rtp_queue_payload_type(_VStreamQ[i]),
        90000,
        _VStreamQ[i]
    );
/*
    akae_rtsp_server_describe_g711a(_Server, registry, _711aStreamQ);
*/
 argv[0] = (AK_voidptr)(intptr_t)i;
argc = 1;
 //argc = 0;
 //argv[argc++] = (AK_voidptr)(intptr_t)i;
//argv[argc++] = param->rtsp_chn[i].vstream;

_VStreamTH[i] = akae_thread_create(
    i == 0 ? "venc_main" : "venc_sub",
    0,              // stack size
    0,              // priority
    _buffer_venc,   // function
    argc,           // argc
    argv            // argv
);

if (_VStreamTH[i] <= 0) {
    ak_print_error_ex("RTSP: failed to create venc thread ch=%d\n", i);
    goto fail;
}
}
/* start server AFTER loop */
if (AK_OK != akae_rtsp_server_start(_Server, port)) {
    port = 8554;
    if (AK_OK != akae_rtsp_server_start(_Server, port)) {
        ak_print_error_ex("RTSP server start failed\n");
        goto fail;
    }
}

return 0;

fail:
if (_Server) {
    akae_rtsp_server_release(_Server);
    _Server = AK_null;
}
if (_Heap) {
    akae_malloc_release(_Heap, AK_null, AK_null);
    _Heap = AK_null;
}
return -1;



	ak_net_get_cur_iface(ac_iface);
	ak_net_get_ip(ac_iface, ac_ip) ;
COLOR_PRINT( COLOR_MODE_BOLD, COLOR_BACK_BLACK, COLOR_FRONT_GREEN, ac_rtsp, LEN_LINK, "***********************************************\n" )
	COLOR_PRINT( COLOR_MODE_BOLD, COLOR_BACK_BLACK, COLOR_FRONT_GREEN, ac_rtsp, LEN_LINK, "*                  RTSP LINK                  *\n" )
	COLOR_PRINT( COLOR_MODE_BOLD, COLOR_BACK_BLACK, COLOR_FRONT_GREEN, ac_rtsp, LEN_LINK, "***********************************************\n" )
	COLOR_PRINT( COLOR_MODE_BOLD, COLOR_BACK_BLACK, COLOR_FRONT_GREEN, ac_rtsp, LEN_LINK, "* MAIN CHANNEL : rtsp://%s:%d/%s\n", ac_ip, port, param->rtsp_chn[0].suffix_name)
	COLOR_PRINT( COLOR_MODE_BOLD, COLOR_BACK_BLACK, COLOR_FRONT_GREEN, ac_rtsp, LEN_LINK, "* SUB CHANNEL  : rtsp://%s:%d/%s\n", ac_ip, port, param->rtsp_chn[1].suffix_name)
	COLOR_PRINT( COLOR_MODE_BOLD, COLOR_BACK_BLACK, COLOR_FRONT_GREEN, ac_rtsp, LEN_LINK, "***********************************************\n" )

	return 0;
}


/**
 * @deprecated >= 1.9.00
 */
int ak_rtsp_start(int index) {
	return 0;
}


/**
 * ak_rtsp_exit - exit rtsp
 * @void
 */
void ak_rtsp_exit(void) {

	AK_int i = 0;
	AK_EXPECT_RETURN (AK_null != _Heap);

	akae_thread_release (_AStreamTH, AK_TRUE);
	_AStreamTH = 0;
	akae_rtp_queue_release_clear (_711aStreamQ);
	_711aStreamQ = AK_null;


	/// 停止视频编码线程。
	for (i = 0; i < RTSP_CHANNEL_NUM; ++i) {
		akae_thread_release (_VStreamTH[i], AK_TRUE);
		_VStreamTH[i] = 0;
		akae_rtp_queue_release_clear (_VStreamQ[i]);
		_VStreamQ[i] = AK_null;
	}


	if (AK_null != _Server) {
		akae_rtsp_server_release (_Server);
		_Server = AK_null;
	}

	/// 释放内存缓冲句柄。
	akae_malloc_release (_Heap, AK_null, AK_null);
	_Heap = AK_null;

}
