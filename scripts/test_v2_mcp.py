"""A real MCP stdio client test; does not contact the Xiaozhi cloud."""
import asyncio
import json
from pathlib import Path
from mcp import ClientSession,StdioServerParameters
from mcp.client.stdio import stdio_client


async def main():
    params=StdioServerParameters(command='bash',args=['/ws/scripts/run_robot_mcp.sh'])
    async with stdio_client(params) as (reader,writer):
        async with ClientSession(reader,writer) as session:
            await session.initialize();listing=await session.list_tools()
            tools=[tool.name for tool in listing.tools]
            status=await session.call_tool('status',{})
            proposal=await session.call_tool('interpret_text',{'text':'查询状态'})
            report={'tools':tools,'status':status.model_dump(mode='json'),'interpret':proposal.model_dump(mode='json')}
            def payload(result):return json.loads(next(c.text for c in result.content if c.type=='text'))
            report['passed']=set(tools)=={'status','interpret_text','submit_skill','pause','cancel'} and payload(status).get('ok',False) and payload(proposal).get('ok',False)
            Path('/ws/output/v2_brain/mcp_acceptance.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
            print(json.dumps(report,ensure_ascii=False),flush=True)
            assert report['passed'],report


if __name__=='__main__':asyncio.run(main())
