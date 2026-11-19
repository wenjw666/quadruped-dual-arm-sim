#!/usr/bin/env python3
"""Apply a reviewed P85 overlay to a NEW copy of the pinned public R0 SDK.

No board access. Refuses unknown source files or an existing destination.
Run with Python 3.8+ on Linux (WSL supported).
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

P85 = Path(__file__).resolve().parents[1]
LOCK = json.loads((P85 / "rt0/hanhai/sdk-lock.json").read_text(encoding="utf-8"))


def read(path):
    return path.read_bytes().replace(b"\r\n", b"\n").decode("utf-8")


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError("SDK anchor missing or ambiguous: " + old[:90])
    return text.replace(old, new, 1)


def patch(relative, text, boot_can_link=False):
    if relative == "Makefile":
        text = once(text, "CLEAN_MODE ?= 0", "# P85: drivers/OS/IPC, no network or timesync demos\noverride CLEAN_MODE := 1")
        return once(text, "C_OBJ     = $(addsuffix .o, $(C_SOURCES))", "include p85.mk\n\nC_OBJ     = $(addsuffix .o, $(C_SOURCES))")
    if relative == "make.sh":
        text = once(text, "#!/bin/bash", "#!/bin/bash\nset -e")
        return once(text, "    # 创建/更新版本控制文件", "    if ! command -v repo >/dev/null 2>&1; then\n        echo 'SDK revisions recorded in p85-sdk-lock.json'\n        exit 0\n    fi\n\n    # 创建/更新版本控制文件")
    if relative.endswith("FreeRTOSConfig.h"):
        return once(text, "#define INCLUDE_vTaskDelayUntil", "#define INCLUDE_uxTaskGetStackHighWaterMark 1\n#define INCLUDE_vTaskDelayUntil")
    if relative.endswith("BswInit.c"):
        text = '#include "p85_board.h"\n' + text
        begin = text.index('    xHandle = TaskCreate((task_func_t )FuncCanAppTask,')
        end = text.index('    xHandle = TaskCreate((task_func_t )FuncComAppTask,', begin)
        text = text[:begin] + '    /* P85 exclusively owns CAN0; CAN/VCan demo tasks disabled. */\n\n' + text[end:]
        text = once(text, '    TSynConfig_start();', '    /* CAN time synchronization disabled in the P85 variant. */\n    p85_board_create_tasks();')
        return text
    if relative.endswith("user_cmd_list.c"):
        pattern = r'SHELL_EXPORT_CMD\(SHELL_CMD_PERMISSION\(0\)\|SHELL_CMD_TYPE\(SHELL_TYPE_CMD_MAIN\)\|SHELL_CMD_DISABLE_RETURN,\s*can, shell_can_Test, Can test\);'
        text, n = re.subn(pattern, '/* CAN test shell command disabled: P85 owns CAN0. */', text)
        if n != 1:
            raise ValueError("CAN shell anchor mismatch")
        return text
    if relative.endswith("isr.cfg"):
        if not boot_can_link:
            return text
        return once(text, '//ISR_DEFINE(RT_INTR_LB_RT_LSP0_CAN1_FUNC_IRQ,',
                    'ISR_DEFINE(RT_INTR_LB_RT_LSP0_CAN1_FUNC_IRQ,')
    if relative.endswith("Can_Irq.c"):
        text = '#include "p85_can_link.h"\n' + text
        return once(text, '    (void) driverState; /* not used yet */',
            '    if (event == FLEXCAN_EVENT_ERROR || event == FLEXCAN_EVENT_BUSOFF)\n'
            '        p85_can_link_fault(u8Instance, u32ErrStatus);\n'
            '    (void) driverState; /* not used yet */')
    if relative.endswith("FlexCAN_Ip.c"):
        if not boot_can_link:
            return text
        text = '#include "p85_can_trace.h"\n' + text
        # Scope each exact replacement to one reviewed function definition.
        def instrument(signature, replacements):
            nonlocal text
            begin = text.index(signature + '\n{')
            end = text.index('\n}', begin) + 2
            body = text[begin:end]
            for old, new in replacements:
                body = once(body, old, new)
            text = text[:begin] + body + text[end:]
        instance = '(pBase == (FLEXCAN_Type *)FLEXCAN_1_BASE ? 1u : 0u)'
        instrument('static Flexcan_Ip_StatusType FlexCAN_InitCtroll(FLEXCAN_Type * pBase, const Flexcan_Ip_ConfigType * Flexcan_Ip_pData)', [
            ('    eResult = FlexCAN_InitRxFifo(pBase, Flexcan_Ip_pData);',
             '    eResult = FlexCAN_InitRxFifo(pBase, Flexcan_Ip_pData);\n'
             '    p85_can_trace(' + instance + ', P85_TRACE_FIFO, eResult, Flexcan_Ip_pData->is_enhanced_rx_fifo_needed);'),
            ('        eResult = FlexCAN_SetMaxMsgBuffNum(pBase, Flexcan_Ip_pData->max_num_mb);',
             '        eResult = FlexCAN_SetMaxMsgBuffNum(pBase, Flexcan_Ip_pData->max_num_mb);\n'
             '        p85_can_trace(' + instance + ', P85_TRACE_MAX_MB, eResult, Flexcan_Ip_pData->max_num_mb);')])
        instrument('static Flexcan_Ip_StatusType FlexCAN_InitController(uint8 Instance, FLEXCAN_Type * pBase, const Flexcan_Ip_ConfigType * Flexcan_Ip_pData)', [
            ('    if (FlexCAN_IsEnabled(pBase))',
             '    p85_can_trace(Instance, P85_TRACE_INIT_ENTER, 0u, Flexcan_Ip_pData->max_num_mb);\n'
             '    if (FlexCAN_IsEnabled(pBase))'),
            ('        eResult = FlexCAN_EnterFreezeMode(pBase);',
             '        eResult = FlexCAN_EnterFreezeMode(pBase);\n'
             '        p85_can_trace(Instance, P85_TRACE_FREEZE, eResult, 0u);'),
            ('            eResult = FlexCAN_Disable(pBase);',
             '            eResult = FlexCAN_Disable(pBase);\n'
             '            p85_can_trace(Instance, P85_TRACE_DISABLE, eResult, 0u);'),
            ('        eResult = FlexCAN_Init(pBase);',
             '        eResult = FlexCAN_Init(pBase);\n'
             '        p85_can_trace(Instance, P85_TRACE_RESET, eResult, 0u);')])
        # Record after timing and runtime state have actually been initialized.
        begin = text.index('Flexcan_Ip_StatusType FlexCAN_Ip_Init_Privileged(')
        end = text.index('\n}', begin) + 2
        body = once(text[begin:end], '    return eResult;',
                    '    p85_can_trace(Flexcan_Ip_u8Instance, P85_TRACE_INIT_RETURN, eResult, 0u);\n'
                    '    return eResult;')
        text = text[:begin] + body + text[end:]
        return text
    if relative.endswith("Can_Ipw.c"):
        text = '#include "p85_can_loopback.h"\n#include "p85_can_link.h"\n#include "p85_can_trace.h"\n' + text
        anchor = '    eRetVal = FlexCAN_Ip_SetStartMode(Can_pControllerConfig->Can_u8ControllerOffset);'
        text = once(text, anchor, anchor + '\n'
                    '    p85_can_trace(Can_pControllerConfig->Can_u8ControllerOffset, P85_TRACE_EXIT_FREEZE, eRetVal, 0u);')
        return once(text, '        CanIf_RxIndication(CanIf_Mailbox, CanIf_PduInfo);',
            '        if (p85_can_link_rx(CanIf_Mailbox->ControllerId, CanIf_Mailbox->CanId,\n'
            '            pReceivedDataBuffer->cs, pReceivedDataBuffer->data, pReceivedDataBuffer->dataLen)) return;\n'
            '        if (CanIf_Mailbox->ControllerId == 0u &&\n'
            '            p85_can_loopback_rx(CanIf_Mailbox->CanId, pReceivedDataBuffer->cs,\n'
            '                pReceivedDataBuffer->data, pReceivedDataBuffer->dataLen)) return;\n'
            '        CanIf_RxIndication(CanIf_Mailbox, CanIf_PduInfo);')
    if relative.endswith("CanIf.c"):
        text = '#include "p85_board.h"\n#include "p85_can_loopback.h"\n#include "p85_can_link.h"\n' + text
        text = once(text, '    CanIf_bTxFlag = 1;', '    if (CanTxPduId == P85_LINK_TOKEN0 || CanTxPduId == P85_LINK_TOKEN1) { p85_can_link_tx_done(CanTxPduId); return; }\n    if (CanTxPduId == P85_LOOPBACK_TOKEN) { p85_can_loopback_tx_done(); return; }\n    CanIf_bTxFlag = 1;')
        anchor = '    PduIdType UserPduId = 0U;'
        text = once(text, anchor, '    if (Mailbox && Mailbox->ControllerId == 0u) {\n        if (PduInfoPtr) p85_board_rx(Mailbox->CanId, PduInfoPtr->SduDataPtr, PduInfoPtr->SduLength);\n        return;\n    }\n' + anchor)
        text = once(text, '    (void)ControllerId;\n}', '    if (ControllerId == 0u) p85_board_bus_fault();\n}')
        text = once(text, 'void CanIf_ControllerErrorStatePassive(uint8 ControllerId, uint16 RxErrorCounter, uint16 TxErrorCounter)\n{\n', 'void CanIf_ControllerErrorStatePassive(uint8 ControllerId, uint16 RxErrorCounter, uint16 TxErrorCounter)\n{\n    if (ControllerId == 0u) p85_board_bus_fault();\n')
        return text
    if relative.endswith("Can_PBcfg.c"):
        text = '#include "p85_board.h"\n' + text
        begin = text.index('static const Can_BaudrateConfigType Can_aBaudrateConfig_Ctrl0[1U]=')
        end = text.index('\n};', begin) + 3
        text = text[:begin] + """/* P85: 80 MHz CAN clock, CBT/FDCBT raw values, 1/5 Mbps, 75% samples.
 * TDC 12 clocks is an initial value, subject to transceiver/bench validation. */
static const Can_BaudrateConfigType Can_aBaudrateConfig_Ctrl0[1U]=
{
    {
        (boolean)FALSE, (boolean)TRUE, (boolean)TRUE,
        { (uint8)38U, (uint8)19U, (uint8)19U, (uint16)0U,
#if (CAN_DUAL_CLOCK_MODE == STD_ON)
          (uint16)0U,
#endif
          (uint8)19U },
        { (uint8)7U, (uint8)3U, (uint8)3U, (uint16)0U,
#if (CAN_DUAL_CLOCK_MODE == STD_ON)
          (uint16)0U,
#endif
          (uint8)3U },
        (uint8)10U, (boolean)TRUE, (uint8)12U
    }
};""" + text[end:]
        # Overflow callback is void(void). Restrict replacement to controller0.
        begin = text.index('static const Can_ControllerConfigType Can_aControllerConfig')
        end = text.index('Can_aBaudrateConfig_Ctrl0,', begin)
        block = text[begin:end]
        block = once(block, '/* Enhanced FIFO Overflow Notification */\n        NULL_PTR,', '/* Enhanced FIFO Overflow Notification */\n        p85_board_bus_fault,')
        text = text[:begin] + block + text[end:]
        if boot_can_link:
            begin = text.index('static const Can_BaudrateConfigType Can_aBaudrateConfig_Ctrl0[1U]=')
            end = text.index('\n};', begin) + 3
            timing = text[begin:end].replace('Ctrl0', 'Ctrl1')
            begin = text.index('static const Can_BaudrateConfigType Can_aBaudrateConfig_Ctrl1[1U]=')
            end = text.index('\n};', begin) + 3
            text = text[:begin] + timing + text[end:]
            text = once(text, 'FLEXCAN_1_BASE,\n        /* Controller Activation */\n        (boolean)FALSE,',
                        'FLEXCAN_1_BASE,\n        /* Controller Activation: R0 physical test only */\n        (boolean)TRUE,')
        return text
    if relative.endswith("FlexCAN_Ip_PBcfg.c"):
        begin = text.index('/* Values for normal baudrate .bitrate */')
        end = text.index('/* Fifo Transfer Type .transfer_type */', begin)
        timing = """/* P85: identical raw timing to Can_aBaudrateConfig_Ctrl0. */
        { (uint8)38U, (uint8)19U, (uint8)19U, (uint16)0U, (uint8)19U },
        { (uint8)7U, (uint8)3U, (uint8)3U, (uint16)0U, (uint8)3U },
        """
        text = text[:begin] + timing + text[end:]
        if boot_can_link:
            begin = text.index('/* Values for normal baudrate .bitrate */', begin + len(timing))
            end = text.index('/* Fifo Transfer Type .transfer_type */', begin)
            text = text[:begin] + timing + text[end:]
        return text
    raise ValueError("Unrecognized SDK patch: " + relative)


def prepare(source, destination, boot_loopback=False, boot_can_link=False):
    source, destination = source.resolve(), destination.resolve()
    if destination.exists() or source == destination or source in destination.parents or destination in source.parents:
        raise ValueError("Destination must be a NEW directory outside the SDK source")
    # Require the assembled manifest checkout, including each nested project.
    # Source anchors are hashed below; HEAD checks also reject mixed releases.
    for project in LOCK["projects"]:
        relative = Path(project["path"]).relative_to("realtime")
        checkout = source / relative
        if not (checkout / ".git").exists():
            raise ValueError("Missing manifest project metadata: " + str(relative))
        revision = subprocess.check_output(
            ["git", "-c", "safe.directory=" + str(checkout), "-C", str(checkout), "rev-parse", "HEAD"],
            text=True).strip()
        if revision != project["revision"]:
            raise ValueError("SDK manifest revision mismatch: " + project["path"])
    for relative, expected in LOCK["source_sha256_lf"].items():
        data = (source / relative).read_bytes().replace(b"\r\n", b"\n")
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError("Unreviewed SDK file/version: " + relative)
    # Exclude Git metadata and artifacts; preserve all nested manifest projects.
    shutil.copytree(source, destination, ignore=shutil.ignore_patterns(".git", "out", "__pycache__"))
    for relative in LOCK["source_sha256_lf"]:
        (destination / relative).write_text(patch(relative, read(source / relative), boot_can_link), encoding="utf-8")
    overlay = destination / "rt_appl/user/p85"
    overlay.mkdir()
    for directory in ("src", "include", "rt0"):
        shutil.copytree(P85 / directory, overlay / directory, ignore=shutil.ignore_patterns("sdk-lock.json"))
    sources = ['src/p85_protocol.c', 'src/p85_control.c', 'src/p85_wire.c',
               'rt0/p85_runtime.c', 'rt0/p85_mcal_tx.c', 'rt0/p85_bst_msgbox.c',
               'rt0/hanhai/p85_board.c', 'rt0/hanhai/p85_hanhai_msgbox.c',
               'rt0/hanhai/p85_can_loopback.c', 'rt0/hanhai/p85_can_link.c',
               'rt0/hanhai/p85_can_trace.c']
    mk = 'P85_DIR := $(TOP_DIR)/rt_appl/user/p85\n'
    mk += 'C_INCLUDES += -I$(P85_DIR)/include -I$(P85_DIR)/rt0 -I$(P85_DIR)/rt0/hanhai\n'
    mk += 'GCCFLAGS += -DP85_CAN_TX_HTH=12 -DP85_CAN_CONFIG_REVIEWED=1\n'
    mk += 'GCCFLAGS += -DP85_BOOT_LOOPBACK=' + ('1' if boot_loopback or boot_can_link else '0') + '\n'
    mk += 'GCCFLAGS += -DP85_BOOT_CAN_LINK=' + ('1' if boot_can_link else '0') + '\n'
    mk += 'LIBS += -lm\n'
    mk += 'C_SOURCES := $(filter-out $(TOP_DIR)/rt_appl/test/driver_test/gmac_test.c $(TOP_DIR)/rt_appl/test/driver_test/vlan_test.c,$(C_SOURCES))\n'
    mk += 'C_SOURCES += ' + ' '.join('$(P85_DIR)/' + s for s in sources) + '\n'
    (destination / "p85.mk").write_text(mk, encoding="utf-8")
    (destination / "p85-sdk-lock.json").write_text(json.dumps(LOCK, indent=2) + '\n', encoding="utf-8")
    for path in destination.rglob("*.sh"):
        path.write_bytes(path.read_bytes().replace(b"\r\n", b"\n"))
    for executable in ("tools/sign_package_open", "bsw/ipc/ipc_app_layer/idl-tools/bin/bst_idl_code_gen"):
        path = destination / executable
        path.chmod(path.stat().st_mode | 0o111)
    print("P85 SDK staged: " + str(destination))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", required=True, type=Path, help="Pinned V1.6 realtime directory")
    parser.add_argument("--dest", required=True, type=Path, help="New isolated realtime directory")
    parser.add_argument("--boot-loopback", action="store_true", help="Startup-only hardware test variant; motor remains OFF")
    parser.add_argument("--boot-can-link", action="store_true", help="CAN0/CAN1 external test; requires both channels wired; CAN1 ownership must be verified")
    args = parser.parse_args()
    prepare(args.sdk, args.dest, args.boot_loopback, args.boot_can_link)
