from pandasai.llm.local_llm import LocalLLM
import streamlit as st
import pandas as pd
from pandasai import SmartDataframe
from langchain_experimental.agents import create_pandas_dataframe_agent
from langchain_experimental.agents.agent_toolkits import create_csv_agent
from langchain_community.chat_models import ChatOllama
from langchain.schema.runnable import RunnablePassthrough
from langchain_core.prompts import PromptTemplate
import time
import matplotlib
matplotlib.use('qt5Agg')
from langchain_core.prompts import ChatPromptTemplate, FewShotChatMessagePromptTemplate
import re


def chat_with_csv(df, query):
    llm = LocalLLM(
        api_base="http://localhost:11434/v1",
        model="llama3"
    )
    pandas_ai = SmartDataframe(df, config={"llm": llm})
    result = pandas_ai.chat(query)
    return result

# @tool("math")
# def math(query : str) -> str:
#     """useful for when you need to answer questions about math"""
#     llm_math_chain = LLMMathChain(llm=llm, verbose=True)
#     return llm_math_chain.run(query)

def chat_ollama(df, query):
    llm = ChatOllama(model="llama3", temperature=0)
    agent = create_pandas_dataframe_agent(
        llm,
        df,
        verbose=True,
        allow_dangerous_code=True,
        )
    
    template = """
    Action: python_repl_ast

    Question: {question}

    --------------------------------------------------------------------------------

    When writing Python code to put into the action input, be careful not to make any syntax errors. 
    Especially, make sure to close any parenthesis that you open."""

    prompt = PromptTemplate.from_template(template)
    st.info("Final Query: " + str(prompt.format(question=input_text)))

    result = agent.run(prompt.format(question=input_text))
    st.info(str(result), type(result))
    return result

examples = [
    {
        "instruction": "Plot the spindle power of MCT 28 over time.",
        "df_head": """Time,MCT28_SPINDLE_POWER,MCT28_TEMP,MCT28_SPINDLE_VIBE,MCT28_PRES,MCT28_PANEL_HUMI,MCT28_PANEL_TEMP,MCT28_MAIN_POWER,MCT20_MAIN_POWER,MCT13_MAIN_POWER,MCT12_MAIN_POWER,MCT7_MAIN_POWER,MCT5_MAIN_POWER,MCT4_MAIN_POWER
        2020-12-01_00:59:42,11,28.34,0.01,44.77,24.89,17.85,3061.6,639.9,1025.5,2178.8,487.4,418.2,1714.7
        2020-12-01_00:59:43,6,28.4,0.01,39.0,24.87,17.86,3063.9,636.6,1023.7,2176.4,490.3,417.7,1723.9
        2020-12-01_00:59:45,10,28.42,0.01,43.36,24.87,17.84,3058.6,641.7,1023.9,2186.5,491.4,416.5,1702.3
        2020-12-01_00:59:45,14,28.39,0.01,43.22,24.85,17.87,3070.8,641.2,1027.4,2190.9,485.8,418.4,1725.4
        2020-12-01_00:59:46,13,28.38,0.01,44.12,24.89,17.86,3074.8,643.3,1032.4,2186.8,485.7,411.8,1710.9
        """,
        "answer": """
        '''python
        import matplotlib.pyplot as plt
        import seaborn as sns
        import pandas as pd
        
        # assuming df is the dataframe
        sns.set()
        
        plt.figure(figsize=(10,6))
        sns.lineplot(x="Time", y="MCT28_SPINDLE_POWER", data=df)
        plt.title("MCT28 SPINDLE POWER OVER TIME")
        plt.xlabel("Time")
        plt.ylabel("Power")
        plt.show()'''
        """
    }
]

def extract_python_code(text):
    # Regular expression pattern to extract content between triple backticks with 'python' as language identifier
    pattern = r"```(.*?)```"

    # re.DOTALL allows the dot (.) to match newlines as well
    match = re.search(pattern, text, re.DOTALL)
    
    if match:
        # Return the matched group, stripping any leading or trailing whitespace
        return match.group(1).strip()
    else:
        return "No Python code found in the input string."
    
def visualize_ollama(df, query):
    llm = ChatOllama(model="llama3", temperature=0)

    example_prompt = ChatPromptTemplate.from_messages(
        [
             ("human", "This is the result of `print(df.head())`: {df_head}"),
             ("human", "{instruction}"),
             ("ai", "{answer}")
        ]
    )

    few_shot_prompt = FewShotChatMessagePromptTemplate(examples=examples, example_prompt=example_prompt)
    prompt = ChatPromptTemplate.from_messages(
        [
            ("system", """Generate the Python code to do given visualization task. Be careful not to make any syntax errors,
             especially the errors associated with parenthesis. Do not execute the code. Just let me know it.
             Enclose the code with ''' ''' to distinguish it from the text.
             Your data input is a pandas dataframe that you can access with df."""),
             few_shot_prompt,
             ("human", "This is the result of `print(df.head())`: {df_head}"),
             ("human", "{instruction}"),
        ]
    )
    question = {
        "instruction" : query,
        "df_head" : str(df.head())
    }
    chain = prompt | llm
    answer = chain.invoke(question)
    st.info(answer.content)

    execution_code = extract_python_code(answer.content)

    
    # template = """

    # --------------------------------------------------------------------------------
    # Generate the Python code to do given visualization task. Be careful not to make any syntax errors,
    # especially the errors associated with parenthesis. Do not execute the code. Just let me know it.
    # --------------------------------------------------------------------------------
    # example
    # --------------------------------------------------------------------------------
    
    # Question: Plot the spindle power of MCT 28 over time.
    # Thought: I need to generate a Python code for a given visualization task.
    # Action: python
    # Action Input:
    # ```
    # import matplotlib.pyplot as plt
    # import seaborn as sns
    # import pandas as pd

    # # assuming df is the dataframe
    # sns.set()

    # plt.figure(figsize=(10,6))
    # sns.lineplot(x="Time", y="MCT28_SPINDLE_POWER", data=df)
    # plt.title("MCT28 SPINDLE POWER OVER TIME")
    # plt.xlabel("Time")
    # plt.ylabel("Power")
    # plt.show()
    # '''

    # Final Answer: 
    # ```
    # import matplotlib.pyplot as plt
    # import seaborn as sns
    # import pandas as pd
    
    # # assuming df is the dataframe
    # sns.set()
     
    # plt.figure(figsize=(10,6))
    # sns.lineplot(x="Time", y="MCT28_SPINDLE_POWER", data=df)
    # plt.title("MCT28 SPINDLE POWER OVER TIME")
    # plt.xlabel("Time")
    # plt.ylabel("Power")
    # plt.show()
    # '''

    # --------------------------------------------------------------------------------
    # Now it's your turn!
    # --------------------------------------------------------------------------------

    # Question: {question}
    # """

    # prompt = PromptTemplate.from_template(template)
    # st.info("Final Query: " + str(prompt.format(question=input_visualize)))

    # result = agent.run(prompt.format(question=input_text))
    # progress_text = "Operation in progress. Please wait. ⏳"
    # my_progress_bar = st.progress(0)
    # status_text = st.empty()

    # for percent_complete in range(0, 101):
    #     time.sleep(0.02)
    #     my_progress_bar.progress(percent_complete)
    #     status_text.text(f"{progress_text} {percent_complete}%")

    # status_text.text("Operation complete! ⌛")
    # st.info(result)
    # return result
    return execution_code


st.set_page_config(layout='wide')
st.title("Multiple CSV ChatApp powered by LLM")

csv_file = st.sidebar.file_uploader("Upload your CSV files", type="csv")

if csv_file:

    st.info("CSV uploaded successfully")
    st.info(csv_file)

    df = pd.read_csv(csv_file)
    st.dataframe(df, use_container_width=True)

    col1, col2 = st.columns([1, 1])
    with col1:
        st.header('Chat with csv')
        st.write('This is the content in the first column.')
        st.info("Chat below")
        input_text = st.text_area("Enter the query", key=1)

        # if input_text:
        #     st.info("Your Query: " + input_text)
        #     result = chat_ollama(csv_file, input_text)

        if input_text:
            if st.button("Chat with csv"):
                st.info("Your Query: " + input_text)
                result = chat_ollama(df, input_text)
                
                st.success('Done')
                st.success(result)

    # Add content to the second column
    with col2:
        st.header('Visualize')
        st.write('This is the content in the second column.')
        st.info("Chat below")
        input_visualize = st.text_area("Enter the query", key=2)
        if input_visualize:
            if st.button("Chat with csv"):
                st.info("Your Query: " + input_visualize)
                execution_code = visualize_ollama(df, input_visualize)
                st.subheader('This is the executed code:')
                st.code(execution_code, language="python", line_numbers=False)
                with st.spinner("Plotting ..."):
                    exec(execution_code)



    

  
    
    ##################################################################3
    # PandasAI 사용시 
    

    # if input_text:
    #     if st.button("Chat with csv"):
    #         st.info("Your Query: " + input_text)
    #         result = chat_with_csv(data, input_text)
    #         st.success(result) 
