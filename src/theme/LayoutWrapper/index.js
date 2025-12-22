import React from 'react';
import ChatWidget from '@theme/ChatWidget';

// This component wraps the entire layout with the chat widget
export default function LayoutWrapper(props) {
  return (
    <>
      {props.children}
      <ChatWidget />
    </>
  );
}