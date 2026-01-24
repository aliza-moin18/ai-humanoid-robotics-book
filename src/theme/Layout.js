import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatbotWidget from '../components/ChatbotWidget/ChatbotWidget';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
        <ChatbotWidget />
      </OriginalLayout>
    </>
  );
}